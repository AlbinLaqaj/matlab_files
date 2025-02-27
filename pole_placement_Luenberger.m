function pole_placement_Luenberger(theta, dtheta, t)
% Inputs:
%    theta: IMU measurement data of angular position [rad]
%    dtheta: IMU measurement data of angular velocity [rad/s]
%    t: sample time [s]
%
% Output:
%    Plots the states estimated using the Luenberger observer for all valid pole combinations.

    % Define the pole ranges
    p1_values = [0.990, 0.991, 0.992]; 
    p2_values = [0.997, 0.998, 0.999];
    
    % System matrices
    A = [1   0   0;
         t   1   0;
         0   0   1];
    C = [0   1   0;
         1   0   1];
    
    % Convert theta and dtheta from radians to degrees
    theta = rad2deg(theta);
    dtheta = rad2deg(dtheta);
    y = [theta(:)'; dtheta(:)'];
    num_data = length(theta);
    
    % Loop over all pole combinations
    for p1 = p1_values
        for p2 = p2_values
            for p3 = p2_values
                
                % Only process if p2 ~= p3
                if p2 ~= p3
                    % Pole combination and observer gain
                    desired_poles = [p1, p2, p3];
                    L = place(A', C', desired_poles)'; 
                    
                    % Initialize states [dtheta_hat; theta_hat; b_hat]
                    x0 = [0; 0; 0]; % Initial states in degrees
                    xk_data = zeros(3, num_data + 1);
                    xk_data(:,1) = x0;
                    
                    % Luenberger observer loop
                    for k = 1:num_data
                        yk = C * xk_data(:,k);
                        xk_data(:,k+1) = A * xk_data(:,k) + L * (y(:,k) - yk);
                    end
                    
                    % Estimated states
                    x_Luenb = xk_data(:, 1:end-1);
                    t_sample = t;
                    time = 0:t_sample:(num_data - 1) * t_sample;
                    
                    % Plot the results
                    figure();
                    
                    % Plot theta comparison
                    subplot(2, 1, 1);
                    plot(time, theta, 'DisplayName', 'Measurement');
                    hold on;
                    plot(time, x_Luenb(2, :), '--', 'DisplayName', 'Estimate');
                    hold off;
                    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
                    ylabel('Amplitude $\theta$ [deg]', 'FontSize', 12, 'Interpreter', 'latex');
                    legend('Location', 'northeast');
                    grid on;
                    
                    % Plot dtheta and bias comparison
                    subplot(2, 1, 2);
                    plot(time, dtheta, 'DisplayName', 'Measurement');
                    hold on;
                    plot(time, x_Luenb(1, :), '--', 'DisplayName', 'Estimate');
                    plot(time, x_Luenb(3, :), '--', 'DisplayName', 'bias');
                    hold off;
                    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
                    ylabel('Amplitude $\dot{\theta}$ [deg/s]', 'FontSize', 12, 'Interpreter', 'latex');
                    legend('Location', 'northeast');
                    grid on;
                    
                    drawnow;
                end
            end
        end
    end
end
