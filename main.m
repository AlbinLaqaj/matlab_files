%% Initialise Workspace

    clc
    clear all
    close all

%% Import parameters

    parameters();

    init_angles = [0;0;170/180*pi];
    init_dangles = [0;0;0];

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