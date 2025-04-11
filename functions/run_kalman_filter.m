function x_Kalm = run_kalman_filter(theta, dtheta, t)
    
% Inputs:
% 
%   theta: Measured values of angular position
%   dtheta: Measured values of angular velocity + bias b
%   t: sample time
% 
% Output:
% 
%   x_Kalm: Estimated states from the observer

    % Initialization
    w0 = deg2rad(0);
    theta0 = deg2rad(0);
    b = deg2rad(0); % durh Beobachtung in der Sensormessung

    x0 = [w0; theta0; b];

    num_data = length(theta);
    xk_data = zeros(3, num_data + 1);
    yk_data = zeros(2, num_data);
    y = [theta'; dtheta'];

    % discretized system matrices
    A = [1   0   0;
         t   1   0;
         0   0   1];

    C = [0   1   0;
         1   0   1];

    % Initial covariance matrix
    pw = 1e1;
    pb = 1e1;
    p11 = t * pw;
    p12 = 1/2 * t^2 * pw;
    p22 = 1/3 * t^3 * pw;
    p33 = t * pb;

    P0 = [p11   p12   0;
          p12   p22   0;
          0      0   p33];

    % discretized covariance matrix for process noise
    qw = 5e-1; %über e2: noisy dtheta
    qb = 1e-4; 

    q11 = t * qw;
    q12 = 1/2 * t^2 * qw;
    q22 = 1/3 * t^3 * qw;
    q33 = t * qb;

    Q = [q11   q12   0;
         q12   q22   0;
         0      0   q33];

    % discretized covariance matrix for measurement noise
    r_theta = 1e-6; % in theta, nicht unter e-6: zu noisy theta und dtheta
    r_dtheta = 1e-4; % in dtheta

    R = 1/t * [r_theta   0;
               0    r_dtheta];

    % Kalman Filter algorithm
    xk_data(:, 1) = x0;
    P = P0;
    for k = 1:num_data
        % Prediction
        xk_data(:, k+1) = A * xk_data(:, k);
        yk_data(:, k) = C * xk_data(:, k+1);
        P = A * P * A' + Q;

        % Update
        K = P * C' / (C * P * C' + R);
        xk_data(:, k+1) = xk_data(:, k+1) + K * (y(:, k) - yk_data(:, k));
        P = P - K * C * P;
    end
    x_Kalm = xk_data;
end
