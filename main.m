%% Initialise Workspace

clc;
clear all;
close all;

%% Load functions

addpath('functions');

%% Load parameters

parameters();
% parameters_new();

%% Initial states

[init_q, init_dq, theta_ref] = initialize_system();

%% Load and preprocess IMU data

filepath = fullfile('..','data','test4.csv');
[theta_IMU, dtheta_IMU] = preprocess_data(filepath);

%% System dynamics
% Sample time
t_sample = 0.005;

% linearized MinSeg
[A, B, C, D] = MinsegLinearization(param);
sys_MS = ss(A, B, C, D);
tf_MS = minreal(tf(sys_MS));

isControllable = rank(ctrb(sys_MS)) == size(A, 2);
isObservable = rank(obsv(sys_MS)) == size(A, 2);

sim('Plant.slx');

% plot_openLoop_theta();
% exportgraphics(gcf, ['C:\Users\albin\OneDrive - Technische Universität Berlin\' ...
    % 'Studium\Bachelor_thesis\Bilder\plot_theta_openLoop.pdf'], 'ContentType', 'vector');
%% Controler

% analyze open loop
[zeros_MS, poles_MS, gains_MS] = zpkdata(tf_MS);

% PID-controler
Kp = -27;
Ki = -25;
Kd = -1;

% % PID-controler
% Kp = -34;
% Ki = -45;
% Kd = -1.2;

% LQR
Q_lqr = diag([1e0, 1e0, 1e3, 1e0, 1e0, 1e2]);
R_lqr = diag([3e5, 3e5]);

% control system
K = lqr(sys_MS, Q_lqr, R_lqr);
K_d = lqrd(A, B, Q_lqr, R_lqr, t_sample);

% % Output y with PID Controller
% sim('MinSeg_sim_PID.slx');
% 
% % Output y with LQR Controller
% sim("MinSeg_sim_LQR.slx");
% 
% plot_controler();
%% Observer

% Luenberger Observer estimates
x_Luenb = run_luenberger_observer(theta_IMU, dtheta_IMU, t_sample);

% Kalman Filter estimates
x_Kalm = run_kalman_filter(theta_IMU, dtheta_IMU, t_sample);

plot_observer(theta_IMU, dtheta_IMU, x_Luenb, x_Kalm);
% plot_measurements(theta_IMU, dtheta_IMU);

% pole_placement_Luenberger(theta_IMU, dtheta_IMU, t_sample);