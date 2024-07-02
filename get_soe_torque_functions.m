% returns a cell array of { function handle, description of the function }
% for soe to torque limit

function ret = get_soe_torque_functions()
    FUNC = 1;
    DESC = 2;

    % assume we get soe as a decimal number <= 1
    ret = cell(2, 2);

    ret{1, FUNC} = @(x) 1000;
    ret{1, DESC} = "no limit";

    ret{2, FUNC} = @(x) x * 100;
    ret{2, DESC} = "soe percent as torque limit";
end