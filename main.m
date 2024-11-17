%% Initialise Workspace

    clc
    clear all
    % close all

%% load functions

    addpath(fullfile(pwd, 'functions')); %pwd = print working directory

%% set parameters

    parameters();

%% Initial Values

    init_phi1 = 0;
    init_phi2 = 0;
    init_theta = 2/180*pi;
    init_dphi1 = 0;
    init_dphi2 = 0;
    init_dtheta = 0;

    init_angles = [init_phi1; init_phi2; init_theta];
    init_dangles = [init_dphi1; init_dphi2; init_dtheta];

%% tf Plant

    % linearized MinSeg
    [A_ms, B_ms, C_ms, D_ms] = MinsegLinearization();
    sys_ms = minreal(ss(A_ms, B_ms, C_ms, D_ms));
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



%% controller design for theta

    [zeros, poles, gains] = zpkdata(tf_ms);

    Q_lqr = diag([1e2 1e2 1e6 1e1 1e1 1e3]);
    R_lqr = diag([1e1 1e1]);

    K = lqr(sys_ms, Q_lqr, R_lqr);

%     % t = 0:0.01:10;
%     % % Anzahl der Nullen (z.B. für die ersten 100 Schritte)
%     % num_zeros = 100;
%     % 
%     % num_ones = length(t) - num_zeros;
%     % u = [zeros(1, num_zeros), 2/180*pi*ones(1, num_ones)];
% 
%     % 1. Modell
    
% 
%     % 2. Modell
% 
% 
% 
% 
%     % % sine wave sim
%     % [u, t] = gensig('sine', 2*pi);
%     % lsim(G_theta_U(1), u, t);
%     % step response
%     % step(G_theta_U,10);

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