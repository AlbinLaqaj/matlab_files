function plot_measurements(theta_IMU, dtheta_IMU)
    % Convert inputs from radians to degrees
    theta_IMU = rad2deg(theta_IMU);
    dtheta_IMU = rad2deg(dtheta_IMU);
    
    % Create time vector based on the length of the measurement data
    t = 0:length(theta_IMU)-1;
    
    % Plot theta measurement data
    figTheta = figure;
    figTheta.Position(3:4) = [800 250];
    plot(t, theta_IMU);
    xlabel('Step $k$ [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\theta$ [deg]', 'FontSize', 12, 'Interpreter', 'latex');
    grid on;
    
    % Plot dtheta measurement data
    figDTheta = figure;
    figDTheta.Position(3:4) = [800 250];
    plot(t, dtheta_IMU);
    xlabel('Step $k$ [-]', 'FontSize', 12, 'Interpreter', 'latex');
    ylabel('Amplitude $\dot{\theta}$ [deg/s]', 'FontSize', 12, 'Interpreter', 'latex');
    grid on;
end
