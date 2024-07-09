tic
numruns = 1;
addpath("csvs in\");
fname = "228_80kmh_40kw_2024.csv";
fout = "csvs out/torquevals_" + fname;
%used to index across multiple optimum lap runs
i = 1;
%est 4.5kwh
%energy for the gear ratios, in kJ
energy = zeros(1,numruns); lost_energy = zeros(1,numruns); 
brake_energy = zeros(1,numruns); current_energy = 0;
cooling_energy = zeros(1,numruns); gear_ratio = zeros(1,numruns);
%calculates min number of laps needed to travel 23 km
track_length = 1.06997; total_laps = ceil(23/track_length);
%distance is 0.25m
dist = 0.25;
%time accelerating/braking in s
taccel = zeros(1,numruns); tbrake = zeros(1,numruns);

%load in data
metadata = [readcell(fname,"Range", [9,1,28,2]); ...
    readcell(fname,"Range",[31,1,55,2])];
metadata = cell2mat(metadata(:,2));
rawin = readtable(fname);
%brakes for yasadu
brake_pos = table2array(rawin(:, 15));
%speed from table is in kmh, convert to m/s
vel = table2array(rawin(:,1))*1000/3600;
last_velocity = 0;
%in seconds
time = table2array(rawin(:,2)); laptime = zeros(1,numruns);
%rotations per minute
rpm = table2array(rawin(:,8));
%empty torque array (Nm)
torque = zeros(1,length(rpm)); avgtorque = zeros(1,numruns);
%empty capacity arrays (Ah)
cap_used = zeros(1,length(rpm)); capacity = zeros(1,numruns);
%empty discharge and voltage arrays
voltage = zeros(1,length(rpm)); discharge = zeros(1,length(rpm));
power = zeros(1,length(rpm));

% longitudinal acceleration
long_acc = table2array(rawin(:,5));
    
%static values from optimum lap
%mass in kg
mass = metadata(22);
%rolling resistance
rr = 0.015; radius = metadata(28);

%air density (kg/m^3) coef drag
rho = metadata(29); cd = metadata(23);  
%coeff of lift (NOTE: usually a downforce is reported as negative cl,
%but in this case im leaving it positive for ease of use
clift = metadata(24); frontal_area = metadata(26);

func = discharge_calcs_test();
syms c pow
    
for z = 1:numruns
    while (i <= length(rpm) && ~(isnan(rpm(i))))

        % for the first entry, the time difference between the start
        % and end of the time segment is the elapsed time
        % and the old velocity needs to be calculated with the longitudinal
        % acceleration
        if (i == 1)
            timediff = time(1);
            last_velocity = vel(1) - (timediff * long_acc(1));
        else
            timediff = time(i) - time(i - 1);
            last_velocity = vel(i - 1);
        end

        %change kinetic energy, kJ (+/-)
        KE = 0.5 * mass * (vel(i)^2 - last_velocity^2) / 1000;
        %aero energy lost, kJ (+)
        AE = dist * 0.5 * rho * cd * frontal_area * last_velocity^2 / 1000;
        %downforce -- used in calculating rolling resistance, N (+)
        DF = 0.5*rho*clift*frontal_area*(last_velocity^2);
        %rolling resistance energy lost, kJ (+)
        RE = dist*rr*(DF+mass*9.81)/1000;
            
        %the current change in energy due to KE, AE and RE
        current_energy = KE + AE + RE;
        lost_energy(z) = lost_energy(z) + AE + RE;

        if current_energy < 0
            brake_energy(z) = brake_energy(z) + KE - AE - RE;
            torque(i) = 0;
            tbrake(z) = tbrake(z) + timediff;
        else
            torque(i) = torquevspower(current_energy/timediff, rpm(i));
            [motor_eff,inv_eff] = efficiency_calcs(torque(i),rpm(i));
            %torquev(i) = torquev2(current_energy, gear_ratio(z));
            cooling_energy(z) = cooling_energy(z) + ... 
            - 2*current_energy + current_energy/motor_eff + ...
            current_energy/inv_eff;
            current_energy =  (current_energy/motor_eff)/inv_eff;
            energy(z) = energy(z) + current_energy;% + ...
            %timediff*free_losses(rpm(i))    ;
            power(i) = current_energy/timediff;
            %electrical calcs
            discharge(i) = subs(func,{c,pow},{capacity(z),power(i)*1000});
            voltage(i) = power(i)*1000/discharge(i);
            cap_used(i) = discharge(i)*timediff/3600;
            capacity(z) = capacity(z) + cap_used(i);
            first = false;
            if torque(i) > 16
                avgtorque(z) = avgtorque(z) + torque(i)*timediff;
                taccel(z) = taccel(z) + timediff;
           end
        end            
        i=i+1;
    end
    laptime(z) = time(i - 1);
    gear_ratio(z) = table2array(rawin(i - 1,7));
    i= i + 65;
end
avgtorque = avgtorque./taccel;
%avgtorquev2 = avgtorquev2./taccel;
%convert energies to kwh
energy = energy*total_laps/3600;
brake_energy = abs(brake_energy*total_laps/3600);
lost_energy = lost_energy*total_laps/3600;

%{
disp("kJ per lap is : " + energy); disp(" ")
disp("Avg Power, Power under accel: " + energy/laptime + ", " + energy/taccel); disp(" ")
disp("Total Energy Used: " + energy); disp(" ")
disp("Laptime, total time: " + laptime + ", " + laptime*total_laps); disp(" ")
disp("Average Torque, %time under torque: " + avgtorque + ", " + taccel/laptime); disp(" ")
%disp(avgtorque);% disp(avgtorquev2)
%}
%plotting
%{
plot(gear_ratio,energy,'b-sq',gear_ratio, brake_energy, 'r-o', gear_ratio, lost_energy,'k^-')
title("Gear Ratio vs. Energy (Top Speed normalized)");  
legend("Energy", "Brake Energy", "Lost Energy")
xlabel("Gear Ratio"); ylabel("Energy (kWh)"); axis([3.2, 4.2,2,3])
figure;
plot(gear_ratio,laptime*total_laps)
title("Gear Ratio vs. Laptime (Top Speed normalized)"); 
xlabel("Gear Ratio"); ylabel("Laptime (s)"); axis tight
toc
%}
%disp(min(voltage(voltage >0)))
M = ["time", "torque", "rpm", "vel", "brake_pos"; time, torque', rpm, vel, brake_pos];

writematrix(M,fout)

toc