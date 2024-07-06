% initializes the cell array used to keep track of the top "num_runs_saved" runs
function ret = initialize_top_runs_array(num_runs_saved, num_channels)
    MAX_RUN_TIME = 1000;

    ret = cell(num_runs_saved, num_channels);

    for i = 1:num_runs_saved
        ret{i, 1} = MAX_RUN_TIME;
    end
end