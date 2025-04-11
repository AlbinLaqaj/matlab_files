function [theta_IMU, dtheta_IMU] = preprocess_data(filepath)

% Inputs:
% 
%    filepath: String specifying the path to the IMU data as .csv file
%
% Outputs:
% 
%    theta_IMU: Vector of calculated tilt angles (radians) based on accelerometer data
%    dtheta_IMU: Vector of angular velocities (radians/second) from gyroscope data

    % Load IMU data
    IMU_data = readmatrix(filepath);

    % Extract relevant data
    acc_y = IMU_data(:, 2);
    acc_z = IMU_data(:, 3);
    gyro_x = IMU_data(:, 4); % opposite direction

    % % Convert acceleration to actual values
    % g = 9.81; % Gravitational acceleration
    % acc_y = acc_y * g;
    % acc_z = acc_z * g;

    % Calculate theta and dtheta
    theta_IMU = deg2rad(90) - atan2(-acc_y, acc_z);
    dtheta_IMU = deg2rad(gyro_x);

end
