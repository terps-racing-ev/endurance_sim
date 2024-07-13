% returns a cell array of { function handle, description of the function }
% for soe to torque limit

function ret = get_soe_torque_functions()
    FUNC = 1;
    DESC = 2;

    NUM_COEFFICIENTS = 70;
    NUM_CONSTANTS = 200;

    NUM_FUNCTIONS = NUM_COEFFICIENTS * NUM_CONSTANTS;

    LOWER_BOUND_COEFFICIENT = 50;
    UPPER_BOUND_COEFFICIENT = LOWER_BOUND_COEFFICIENT + NUM_COEFFICIENTS - 1;

    LOWER_BOUND_CONSTANT = 0;
    UPPER_BOUND_CONSTANT = LOWER_BOUND_CONSTANT + NUM_CONSTANTS - 1;

    % assume we get soe as a decimal number <= 1
    ret = cell(NUM_FUNCTIONS, 2);

    % each function is a n x 3 matrix describing a piecewise linear function
    % the first element of each row is the upper bound for the soe that the 
    % piece applies to. The second element of each row is the coefficient
    % for that piece, and the third is the constant.
    k = 1;
    for i = LOWER_BOUND_COEFFICIENT:UPPER_BOUND_COEFFICIENT
        for j = LOWER_BOUND_CONSTANT:UPPER_BOUND_CONSTANT
            ret{k, FUNC} = [1, i, j];
            ret{k, DESC} = string(i) + " x soe + " + string(j);
            k = k + 1;
        end
    end
end