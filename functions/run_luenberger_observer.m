function x_Luenb = run_luenberger_observer(theta, dtheta, t)

% Inputs:
% 
%   theta: Measured values of angular position
%   dtheta: Measured values of angular velocity + bias b
%   t: sample time
% 
% Output:
% 
%   x_Luenb: Estimated states from the observer

    % Initialization
    w0 = 0;
    theta0 = deg2rad(0);
    b = deg2rad(0);

    x0 = [w0; theta0; b];
    num_data = length(theta);
    xk_data = zeros(3, num_data + 1);
    yk_data = zeros(2, num_data);
    y = [theta'; dtheta'];

    % discretized System matrices
    A = [1 0 0;
        t 1 0;
        0 0 1]; % discretized

    C = [0 1 0;
         1 0 1];

    % Observer gain design
    desired_poles = [0.990, 0.997, 0.999]; % nahe 0 -> -inf
    L = place(A', C', desired_poles)';

    % Luenberger algorithm
    xk_data(:, 1) = x0;
    for k = 1:num_data
        yk_data(:,k) = C * xk_data(:, k);
        xk_data(:, k+1) = A * xk_data(:, k) + L * (y(:, k) - yk_data(:,k));
    end

    x_Luenb = xk_data;
end
