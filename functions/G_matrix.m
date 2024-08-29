function GM = G_matrix(q)

    % global parameters
    global M g l

    % extract variables from q
    theta = q(3);

    % Calculate the elements of the G(q) matrix
    GM = [0; 0; -M*g*l*sin(theta)];
end
