%% Initialise Workspace

clc
clear all
close all

%% load functions

addpath(fullfile(pwd, 'functions')); %pwd = print working directory

%% set parameters

parameters();

%% Initial Values

init_phi1 = 0;
init_phi2 = 0;
init_theta = 2 * (pi/180);
init_dphi1 = 0;
init_dphi2 = 0;
init_dtheta = 0;

init_angles = [init_phi1; init_phi2; init_theta];
init_dangles = [init_dphi1; init_dphi2; init_dtheta];

% t_sample = 0;

%% tf Plant

% linearized MinSeg
[A_ms, B_ms, C_ms, D_ms] = MinsegLinearization();
sys_ms = ss(A_ms, B_ms, C_ms, D_ms);
tf_ms = minreal(tf(sys_ms));

% dc motor, tau = tf_tau_U*theta + tf_tau_dphi * dphi
s = tf('s');

% U -> tau
tf_tau_U = (eta*N*Km)/(L*(s+R/L));
ss_tau_U = ss(tf_tau_U);

% dphi -> tau (for phi1 and phi2)
tf_tau_dphi = (-eta*N*Km^2)/(L*(s+R/L));
ss_tau_dphi = ss(tf_tau_dphi);

% Model 1
G_theta_U = tf_ms(3,1)*tf_tau_U;  % set dphi = 0
ss_theta_U = ss(G_theta_U);

% Model 2

%% lqr controller design for theta

[zeros, poles, gains] = zpkdata(tf_ms);

% beginning weights
% Q_lqr = diag([1e2 1e2 1e5 1e1 1e1 1e3]);
% R_lqr = diag([2e2 2e2]);

Q_lqr = diag([1e1 1e1 6e5 1e1 1e1 1e2]);
R_lqr = diag([2e3 2e3]);

K = lqr(sys_ms, Q_lqr, R_lqr);
% help lqrd % discrete lqr

% %% load sensor data
% 
% % Load the CSV file containing sensor data
% data = readmatrix('..\data\sensor_data.csv');
% 
% % Assign columns to respective variables
% Acc_X = data(:, 1); % Not used in this calculation
% Acc_Y = data(:, 2); % Radial acceleration (a_r)
% Acc_Z = data(:, 3); % Tangential acceleration (a_t)
% Gyro_X = data(:, 4); % Gyroscope X value (changing angular velocity)
% Gyro_Y = data(:, 5); % Gyroscope Y value
% Gyro_Z = data(:, 6); % Gyroscope Z value 
% 
% % Compute the angle theta
% % Formula: theta = arctan(2 * (-a_r / a_t))
% theta = atan(2 * (-Acc_Y ./ Acc_Z)); % Theta in radians
% 
% % Convert radians to degrees (optional)
% theta_deg = rad2deg(theta);
% 
% % Calculate bias for gyroscope values
% % Bias is computed as the mean of each gyroscope axis
% Gyro_X_bias = mean(Gyro_X);
% Gyro_Y_bias = mean(Gyro_Y);
% Gyro_Z_bias = mean(Gyro_Z);
% 
% % Centered gyroscope values (bias removed)
% Gyro_X_centered = Gyro_X - Gyro_X_bias;
% Gyro_Y_centered = Gyro_Y - Gyro_Y_bias;
% Gyro_Z_centered = Gyro_Z - Gyro_Z_bias;
% 
% % Display results
% disp('Theta (in degrees):');
% disp(theta_deg);
% 
% disp('Gyroscope Bias:');
% disp(['Gyro_X Bias: ', num2str(Gyro_X_bias)]);
% disp(['Gyro_Y Bias: ', num2str(Gyro_Y_bias)]);
% disp(['Gyro_Z Bias: ', num2str(Gyro_Z_bias)]);
% 
% % Plot theta values
% figure;
% plot(theta_deg, 'LineWidth', 1.5);
% xlabel('Measurement Index');
% ylabel('\theta (degrees)');
% title('Calculation of \theta from Accelerometer Data');
% grid on;
% 
% % Plot original and centered gyroscope Z values
% figure;
% subplot(2, 1, 1);
% plot(Gyro_Z, 'LineWidth', 1.5);
% title('Original Gyroscope Z Values');
% xlabel('Measurement Index');
% ylabel('Gyro Z (deg/s)');
% grid on;
% 
% subplot(2, 1, 2);
% plot(Gyro_Z_centered, 'LineWidth', 1.5);
% title('Bias-Corrected Gyroscope Z Values');
% xlabel('Measurement Index');
% ylabel('Gyro Z (deg/s)');
% grid on;



%% Luenberger observer

A_L = [0 0 0; 0 1 0; 0 0 0];
C_L = [0 1 0; 1 0 1];

% %% Run simulation 
% 
%     states = sim('model_simulation.slx');
% 
% %% Plot the states
% 
% figure(1)
% subplot(3,1,1)
% plot(states.states.Time,states.states.Data(:,1))
% title('phi1')
% subplot(3,1,2)
% plot(states.states.Time,states.states.Data(:,2))
% title('phi2')
% subplot(3,1,3)
% plot(states.states.Time,states.states.Data(:,3))
% title('theta')
% 
% figure(2)
% subplot(3,1,1)
% plot(states.states.Time,states.states.Data(:,4))
% title('dphi1')
% subplot(3,1,2)
% plot(states.states.Time,states.states.Data(:,5))
% title('dphi2')
% subplot(3,1,3)
% plot(states.states.Time,states.states.Data(:,6))
% title('dtheta')