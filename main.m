%% Initialise Workspace

    clc
    clear all
    close all

%% Initialise functions

    addpath(fullfile(pwd, 'functions')); %pwd = print working directory

%% Set parameters

    parameters();

    init_angles = [0;0;2/180*pi];
    init_dangles = [0;0;0];

%% Plant linearization
    
    % tf MinSeg
    [A_ms, B_ms, C_ms, D_ms] = MinsegLinearization();
    sys_ms = ss(A_ms, B_ms, C_ms, D_ms);
    tf_ms = minreal(tf(sys_ms));

    % tf dc motor
    s = tf('s');
    tf_U = (eta*N*Km)/(L*(s+R/L));
    tf_dphi = (-eta*N*Km^2)/(L*(s+R/L)); % tau = tf_U*theta + tf_dphi * dphi
    G_dc = [tf_U tf_dphi];

%% controller design for theta

    [zeros, poles, gains] = zpkdata(tf_ms);

    % t = 0:0.01:10;
    % % Anzahl der Nullen (z.B. für die ersten 100 Schritte)
    % num_zeros = 100;
    % 
    % num_ones = length(t) - num_zeros;
    % u = [zeros(1, num_zeros), 2/180*pi*ones(1, num_ones)];
 
    % 1. Modell
    G_theta_U = tf_ms(3,1)*tf_U;  % set dphi = 0

    % 2. ModelL
    
    
    

    % % sine wave sim
    % [u, t] = gensig('sine', 2*pi);
    % lsim(G_theta_U(1), u, t);
    % step response
    % step(G_theta_U,10);

%% Run simulation 

    states = sim('model_simulation.slx');

%% Plot the states

figure(1)
subplot(3,1,1)
plot(states.states.Time,states.states.Data(:,1))
title('phi1')
subplot(3,1,2)
plot(states.states.Time,states.states.Data(:,2))
title('phi2')
subplot(3,1,3)
plot(states.states.Time,states.states.Data(:,3))
title('theta')

figure(2)
subplot(3,1,1)
plot(states.states.Time,states.states.Data(:,4))
title('dphi1')
subplot(3,1,2)
plot(states.states.Time,states.states.Data(:,5))
title('dphi2')
subplot(3,1,3)
plot(states.states.Time,states.states.Data(:,6))
title('dtheta')