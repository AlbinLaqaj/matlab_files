function plot_observer(theta_IMU, dtheta_IMU, x_Luenb, x_Kalm)

% Inputs: 
% 
%    theta_IMU: IMU meassurment data of theta
%    dtheta_IMU: IMU meassurment data of derivative of theta
%    x_Luenb: states with Luenberger observer
%    x_Kalm: states with Kalman Filter

% Output:
%    plots the estimates of Kalman Filter and Luenberger Observer

    % Convert inputs from radians to degrees
    theta_IMU = rad2deg(theta_IMU);
    dtheta_IMU = rad2deg(dtheta_IMU);
    x_Luenb = rad2deg(x_Luenb);
    x_Kalm = rad2deg(x_Kalm);

    t11 = 0:1:1e4-1;
    
    % plot theta
    
    figure(11); %eig 11
    fig11 = figure(11);
    fig11.Position(3:4) = [800 500];
    
    % comparison theta and Luenberger estimate theta
    subplot(2,1,1)
    plot(t11, theta_IMU, 'DisplayName', 'Measurement');
    hold on;
    plot(t11, x_Luenb(2, 1:end-1),'Color', [1, 0.5, 0], ...
        'DisplayName', 'Luenberger estimate', 'LineWidth', 1.2);
    hold off
    
    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\theta$ [deg]', ...
        'FontSize', 12, 'Interpreter', 'latex');
    legend('Location', 'northeast');
    grid on;
    
    % comparison theta and Kalman estimate theta
    subplot(2,1,2)
    plot(t11, theta_IMU, 'DisplayName', 'Measurement');
    hold on;
    plot(t11, x_Kalm(2, 1:end-1),'Color', [1, 0.5, 0], ...
        'DisplayName', 'Kalman estimate', 'LineWidth', 1.2);
    hold off
    
    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\theta$ [deg]', ...
        'FontSize', 12, 'Interpreter', 'latex');
    legend('Location', 'northeast');
    grid on;
    
    % plot theta_dot and bias
    figure(12); %eig 12
    fig12 = figure(12);
    fig12.Position(3:4) = [800 500];
    
    subplot(2,1,1)
    plot(t11, dtheta_IMU, 'DisplayName', 'Measurement');
    hold on
    plot(t11, x_Luenb(1, 1:end-1), '--', 'DisplayName', 'Luenberger estimate', 'LineWidth', 1.5);
    plot(t11, x_Luenb(3, 1:end-1), '--', 'DisplayName', 'bias estimate', 'LineWidth', 1.5);
    hold off
    
    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\dot{\theta}$ [deg/s]', ...
        'FontSize', 12, 'Interpreter', 'latex');
    legend('Location', 'east');
    grid on;
    
    subplot(2,1,2)
    plot(t11, dtheta_IMU, 'DisplayName', 'Measurement');
    hold on
    plot(t11, x_Kalm(1, 1:end-1), '--', 'DisplayName', 'Kalman estimate', 'LineWidth', 1.5);
    plot(t11, x_Kalm(3, 1:end-1), '--', 'DisplayName', 'bias estimate', 'LineWidth', 1.5);
    hold off
    
    xlabel('Step k [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\dot{\theta}$ [deg/s]', ...
        'FontSize', 12, 'Interpreter', 'latex');
    legend('Location', 'east');
    grid on;

end