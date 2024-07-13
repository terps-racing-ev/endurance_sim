function torque_limit = get_torque_limit(soe_to_torque_matrix,soe)
    UPPER_BOUND = 1;
    COEFFICIENT = 2;
    CONSTANT = 3;
    num_rows = size(soe_to_torque_matrix, 1);

    i = 1;

    % find the coefficient and constant to use for the given soe
    while i <= num_rows && soe > soe_to_torque_matrix(i, UPPER_BOUND)
        i = i + 1;
    end

    c = soe_to_torque_matrix(i, COEFFICIENT);
    co = soe_to_torque_matrix(i, CONSTANT);

    torque_limit = c * soe + co;
end