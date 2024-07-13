% Lap Time sim where a torque limit is set by SoE
% Assuming you have the output of gr_vs_power_out (Ryan's sim)
% Yasadu De Silva

tic
% constants
% indices for function array
FUNC = 1;
DESC = 2;

% save the top x runs
NUM_RUNS_SAVED = 5;

% filename of output from ryan's sim
INPUT_FILENAME = "csvs out/torquevals_228_80kmh_57kw_2024";
OPTIMUMLAP_INPUT_FILENAME = "csvs in/228_80kmh_57kw_2024";
TEST_DESC = "228_80kmh_57kw_2024";

% total energy in battery pack
TOTAL_KJ = 25194.24;
MAX_LAPS = 22;

% vehicle characteristics
metadata = [readcell(OPTIMUMLAP_INPUT_FILENAME,"Range", [9,1,28,2]); ...
    readcell(OPTIMUMLAP_INPUT_FILENAME,"Range",[31,1,55,2])];
metadata = cell2mat(metadata(:,2));

% mass, kg
MASS = metadata(22);

% drag coefficient
CD = metadata(23);

% air density (kg/m^3)
RHO = metadata(29);

% coefficient of lift (leaving downforce as positive instead of negative)
CLIFT = metadata(24);

% frontal area (m^2)
FRONTAL_AREA = metadata(26);

% rolling drag 
RR = .015;

optimumlap_rawin = readcell(OPTIMUMLAP_INPUT_FILENAME, "Range", [70, 7, 71, 8]);

% gear ratio
GEAR_RATIO = optimumlap_rawin{1, 1};

% rolling radius (m)
ROLLING_RADIUS = metadata(28);

% drivetrain efficiency
DRIVETRAIN_EFFICIENCY = 0.95;

% stop run once it dips below this velocity
MIN_VELOCITY = 0.1;

% stop run once it dips below this voltage
MIN_VOLTAGE = 307;

% only considerruns with this many laps to save
MIN_LAPS_THRESHHOLD = MAX_LAPS;


% list of soe_to_torque function handles to test
soe_to_torque_limit_functions = get_soe_torque_functions();
num_functions = size(soe_to_torque_limit_functions, 1);

% distance of a segment, in meters
SEGMENT_DIST = 0.25;

% ryan's sim output
rawin = readtable(INPUT_FILENAME);
reference_lap_elapsed_time = table2array(rawin(:,1));
reference_lap_requested_torque = table2array(rawin(:,2));
reference_lap_segment_end_velocity = table2array(rawin(:,4));
reference_lap_brake_pos = table2array(rawin(:,5));
num_segments_in_lap = length(reference_lap_elapsed_time);

% velocities from this sim at the end of each segment
segment_end_velocities = zeros(1, num_segments_in_lap * MAX_LAPS);

% energy used over each segment (NOT CUMULATIVE)
segment_end_energy_used = zeros(1, num_segments_in_lap * MAX_LAPS);

% cumulative energy used by this point in the run
segment_end_cumulative_energy_used = zeros(1, num_segments_in_lap * MAX_LAPS);

% torque used at the start of this segment
segment_torque_used = zeros(1, num_segments_in_lap * MAX_LAPS);

% current discharged in each segment
segment_discharge = zeros(1, num_segments_in_lap * MAX_LAPS);

% power used from battery pack in each segment
segment_power_drawn_from_battery = zeros(1, num_segments_in_lap * MAX_LAPS);

% voltage in battery pack at each segment
segment_voltage = zeros(1, num_segments_in_lap * MAX_LAPS);

% time taken in each segment
time_taken_each_segment = zeros(1, num_segments_in_lap * MAX_LAPS);

% time taken in each lap
time_taken_each_lap = zeros(1, MAX_LAPS);

% number of data channels to save in top runs array
NUM_DATA_CHANNELS = 12;

% top x runs
top_runs = initialize_top_runs_array(NUM_RUNS_SAVED, NUM_DATA_CHANNELS);
% indices for top x runs cell array
AVG_LAP_TIME = 1;
END_VELOCITIES = 2;
ENERGY_USED = 3;
CUMULATIVE_ENERGY_USED = 4;
TORQUE = 5;
DISCHARGE = 6;
POWER_DRAWN_FROM_BATTERY = 7;
VOLTAGE = 8;
TIME_TAKEN_EACH_SEGMENT = 9;
TIME_TAKEN_EACH_LAP = 10;
DESCRIPTION = 11;
NUM_LAPS_COMPLETED = 12;


% whether a folder has been made for this run of the sim
folder_made = false;

folder_name = "";

func = discharge_calcs_test();
syms c pow;

for i = 1:num_functions
    % current soe -> torque limit function
    soe_to_torque_function = soe_to_torque_limit_functions{i, FUNC};

    % name/description of current soe -> torque function
    function_desc = soe_to_torque_limit_functions{i, DESC};

    % continue the while loop
    do_continue = true;

    % current distance segment (total)
    current_segment = 1;

    % current track segment (loops every lap)
    current_segment_on_track = 1;

    % current lap number
    current_lap_number = 1;

    % current energy
    current_energy = TOTAL_KJ;

    % energy used this segment
    energy_used_this_segment = 0;

    % time elapsed in current lap
    current_lap_time = 0;

    capacity = 0;

    segment_end_velocities = zeros(1, num_segments_in_lap * MAX_LAPS);

    % zero out everything
    segment_end_velocities = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_end_energy_used = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_end_cumulative_energy_used = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_torque_used = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_discharge = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_power_drawn_from_battery = zeros(1, num_segments_in_lap * MAX_LAPS);
    segment_voltage = zeros(1, num_segments_in_lap * MAX_LAPS);
    time_taken_each_segment = zeros(1, num_segments_in_lap * MAX_LAPS);
    time_taken_each_lap = zeros(1, MAX_LAPS);


    while do_continue && current_lap_number <= MAX_LAPS
        energy_used_this_segment = 0;
        
        last_velocity = 0;
        if current_segment == 1
            last_velocity = reference_lap_segment_end_velocity(1);
        else
            last_velocity = segment_end_velocities(1, current_segment - 1);
        end

        % approximate the time taken in this segment by dividing 
        % distance of segment by velocity at end of last segment (inaccurate but
        % not by much)
        time_taken_this_segment = SEGMENT_DIST / last_velocity;
        time_taken_each_segment(1, current_segment) = time_taken_this_segment;

        % increase current lap time
        current_lap_time = current_lap_time + time_taken_this_segment;


        % get driver torque request from this segment
        torque_request = reference_lap_requested_torque(current_segment_on_track);

        % if torque request is 0 -> no energy will be used, we have rolling
        % and aero losses and (possibly) some braking
        if torque_request == 0
            % calculate rolling resistance and aero losses over this
            % distance segment (both in J)
            aero_loss = SEGMENT_DIST * 0.5 * RHO * CD * FRONTAL_AREA * last_velocity^2;
            downforce = 0.5 * RHO * CLIFT * FRONTAL_AREA * last_velocity^2;
            rolling_loss = SEGMENT_DIST * RR * (downforce + MASS * 9.81);

            % kinetic energy at end of last timestep (J)
            last_kinetic_energy = 0.5 * MASS * (last_velocity^2);

            % kinetic energy at end of this timestep
            this_kinetic_energy = last_kinetic_energy - aero_loss - rolling_loss;

            % calculate the velocity with aero and rolling losses
            this_velocity_provisional = sqrt(this_kinetic_energy * 2 / MASS);

            % if the driver braked down to a lower velocity than this for
            % this segment during the reference lap, then assume they'll do 
            % so for this lap
            if reference_lap_brake_pos(current_segment_on_track) > 0 ...
                    && reference_lap_segment_end_velocity(current_segment_on_track) < this_velocity_provisional

                this_velocity_provisional = reference_lap_segment_end_velocity(current_segment_on_track);
            end

            if this_velocity_provisional < 0
                this_velocity_provisional = 0;
            end

            % no energy has been used from the battery pack (soe will
            % not change)
            segment_end_velocities(1, current_segment) = this_velocity_provisional;
            segment_end_energy_used(1, current_segment) = 0;

            segment_torque_used(1, current_segment) = 0;

            segment_power_drawn_from_battery(1, current_segment) = 0;
        else
            torque_limit = get_torque_limit(soe_to_torque_function, current_energy / TOTAL_KJ);

            % set torque based on limit and requested torque
            torque = torque_request;
            if torque_limit < torque_request
                torque = torque_limit;
            end

            segment_torque_used(1, current_segment) = torque;

            % aero and rolling loss in J
            aero_loss = SEGMENT_DIST * 0.5 * RHO * CD * FRONTAL_AREA * last_velocity^2;
            downforce = (0.5 * RHO * CLIFT * FRONTAL_AREA * last_velocity^2);
            rolling_loss = SEGMENT_DIST * RR * (downforce + MASS * 9.81);

            % figure out motor velocity (rad/s)
            motor_angular_velocity = last_velocity * GEAR_RATIO / (ROLLING_RADIUS);

            % figure out rotational power (watts)
            power = torque * motor_angular_velocity;

            % figure out rotational energy (kiloJoules)
            rotational_energy = power * time_taken_this_segment / 1000;

            motor_rpm = (motor_angular_velocity / (2 * pi)) / 60;

            % last segment kinetic energy in J
            last_kinetic_energy = 0.5 * MASS * last_velocity^2;

            this_kinetic_energy = last_kinetic_energy ...
                                  + ((rotational_energy * 1000) * DRIVETRAIN_EFFICIENCY) ...
                                  - aero_loss ...
                                  - rolling_loss;
            
            this_velocity = sqrt(2 * this_kinetic_energy / MASS);

            segment_end_velocities(1, current_segment) = this_velocity;

            % energy consumed from battery is affected by motor and
            % inverter efficiency
            [motor_eff,inv_eff] = efficiency_calcs(torque,motor_rpm);
            energy_used_this_segment = (rotational_energy / motor_eff) / inv_eff;

            segment_power_drawn_from_battery(1, current_segment) = power / motor_eff / inv_eff;

            segment_end_energy_used(1, current_segment) = energy_used_this_segment;

            % subtract energy used from current energy
            current_energy = current_energy - energy_used_this_segment;
            
        end
        
        % if segment_power_drawn_from_battery(1, current_segment) ~= 0
        %     segment_discharge(1, current_segment) = subs(func,{c,pow},{capacity,segment_power_drawn_from_battery(i, current_segment)});
        %     segment_voltage(1, current_segment) = segment_power_drawn_from_battery(i, current_segment)/segment_discharge(i, current_segment);
        % else
        %     segment_discharge(1, current_segment) = 0;
        %     segment_voltage(1, current_segment) = segment_voltage(1, current_segment - 1);
        % end
        % cap_used = segment_discharge(1, current_segment)*time_taken_this_segment/3600;
        % capacity = capacity + cap_used;

        segment_end_cumulative_energy_used(1, current_segment) = TOTAL_KJ - current_energy;

        % increment current_segment_on_track
        current_segment_on_track = current_segment_on_track + 1;
        
        % once we go past the end of a lap, go back to the first segment
        if current_segment_on_track > num_segments_in_lap
            current_segment_on_track = 1;

            % record current lap time
            time_taken_each_lap(1, current_lap_number) = current_lap_time;

            % reset lap time
            current_lap_time = 0;

            % and increment current_lap_number
            current_lap_number = current_lap_number + 1;
        end
        
        % increment current_segment
        current_segment = current_segment + 1;

        % stop the lap once we run out of energy
        if current_energy <= 0
            do_continue = false;
        end

        % or once we come to a stop
        if segment_end_velocities(1, current_segment - 1) < MIN_VELOCITY
            do_continue = false;
        end

        % if segment_voltage(1, current_segment - 1) < MIN_VOLTAGE
        %     do_continue = false;
        % end
    end

    % if the above while loop ended because we completed the run
    % or if we failed the run but ended up completing a minimum number of
    % laps
    if do_continue || current_lap_number > MIN_LAPS_THRESHHOLD
        average_lap_time = mean(time_taken_each_lap(1, :));

        % find the run with the slowest lap time and replace it with this
        % run if this run is faster
        max_lap_time = 0;
        max_lap_time_index = 1;
        for k = 1:NUM_RUNS_SAVED
            if top_runs{k, AVG_LAP_TIME} > max_lap_time
                max_lap_time = top_runs{k, AVG_LAP_TIME};
                max_lap_time_index = k;
            end
        end

        if max_lap_time > average_lap_time
            top_runs{max_lap_time_index, AVG_LAP_TIME} = average_lap_time;

            top_runs{max_lap_time_index, END_VELOCITIES} = segment_end_velocities(1,:);
            top_runs{max_lap_time_index, ENERGY_USED} = segment_end_energy_used(1,:);
            top_runs{max_lap_time_index, CUMULATIVE_ENERGY_USED} = segment_end_cumulative_energy_used(1,:);
            top_runs{max_lap_time_index, DISCHARGE} = segment_discharge(1,:);
            top_runs{max_lap_time_index, TORQUE} = segment_torque_used(1,:);
            top_runs{max_lap_time_index, POWER_DRAWN_FROM_BATTERY} = segment_power_drawn_from_battery(1,:);
            top_runs{max_lap_time_index, VOLTAGE} = segment_voltage(1,:);
            top_runs{max_lap_time_index, TIME_TAKEN_EACH_SEGMENT} = time_taken_each_segment(1,:);
            top_runs{max_lap_time_index, TIME_TAKEN_EACH_LAP} = time_taken_each_lap(1,:);
            top_runs{max_lap_time_index, DESCRIPTION} = function_desc(1,:);
            top_runs{max_lap_time_index, NUM_LAPS_COMPLETED} = current_lap_number - 1;
        end
    end
    
end

toc

"Saving Top Runs..."
% save the top runs into excel spreadsheets
for i = 1:NUM_RUNS_SAVED
    if top_runs{i, AVG_LAP_TIME} < 1000
        top_runs{i, DESCRIPTION}
        M = [top_runs{i, TORQUE}', top_runs{i, END_VELOCITIES}', top_runs{i, TIME_TAKEN_EACH_SEGMENT}', top_runs{i, ENERGY_USED}', top_runs{i, CUMULATIVE_ENERGY_USED}', top_runs{i, VOLTAGE}', top_runs{i, POWER_DRAWN_FROM_BATTERY}'];
        M_titles = ["torque", "vel_end_of_step (m/s)", "time elapsed at end of segment (s)", "energy_used_per_step (kJ)", "energy_used_cumulative (kJ)", "voltage (V)", "power drawn from battery (W)"];



        % number of laps completed (including fractional lap)
        % laps_completed = (current_lap_number - 1) + (current_segment_on_track / num_segments_in_lap);

        % make a folder to save the sim results if one hasn't been made yet

        if folder_made == false
            d = string(datetime('now').Day);
            m = string(datetime('now').Month);
            y = string(datetime('now').Year);
            h = string(datetime('now').Hour);
            min = string(datetime('now').Minute);

            folder_name = "tl_sim_results_" + TEST_DESC + "_" + m + "." + d + "." + y + "-" + h + "." + min;
            mkdir(folder_name);
            folder_made = true;
        end

        filename = folder_name + "/" + top_runs{i, DESCRIPTION} + ".xlsx";

        N = {"Function used", top_runs{i, DESCRIPTION}; ...
            "Average Lap Time", top_runs{i, AVG_LAP_TIME}; ...
            "Number of laps completed", top_runs{i, NUM_LAPS_COMPLETED}};

        L = {"Time taken each lap"};
        K = [(1:MAX_LAPS)', top_runs{i, TIME_TAKEN_EACH_LAP}'];

        writecell(N, filename, 'Sheet', "Summary", 'Range', 'A1');
        writecell(L, filename, 'Sheet', "Summary", 'Range', 'A6');
        writematrix(K, filename, 'Sheet', 'Summary', 'Range', 'A7');

        writematrix(M_titles, filename, 'Sheet', 'Segment Values', 'Range', 'A1');
        writematrix(M, filename, 'Sheet', 'Segment Values', 'Range', 'A2');
    end
end