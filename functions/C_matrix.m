function CM = C_matrix(x)

    % global parameters
    global r M l

    % extract variables from x = [q, q_dot]
    theta = x(3);
    theta_dot = x(6);

    % Calculate the elements of the C(q, q_dot) matrix
    CM(1,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(2,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(3,3) = -M*l*r*theta_dot*sin(theta);
end

%change input in to x
