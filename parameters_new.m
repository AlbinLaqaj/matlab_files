% Wheel and chassis parameters

r = 0.044; % Outer radius of the wheel, [r] = m
m = 0.018 + 0.02; % m_rad = 0.018 + m_dc = 0.02, [m] = kg
M = 0.2753; % Chassis mass, [M] = kg
% l = 0.056; % Distance between the connection line and the chassis COM, [l] = m
h = 0.034; % Chassis depth, [h] = m
% b = 0.120; % Chassis width, [b] = m
g = 9.81; % Gravitational acceleration, [g] = m/s^2
% I = M/12*(h^2 + b^2) + M*l^2; % Moment of inertia of the chassis, [I] = kg*m^2
J = 1/2*m*r^2; % Moment of inertia of the wheel, [J] = kg*m^2
d = 0; % friction between wheels and ground
c = 0.0022; % Friction between chassis and axle

%% Motor parameters

N = 150;   % Gear ratio, [N] = 1
R = 10.1; % Armature resistance, [R] = Ohm
L = 3.1e-3; % Armature inductance, [L] = H
Km = 2.38e-3; % Motor constant, [Km] = V/rad/s
eta = 0.3; % Gear efficiency, [eta] = 1
tau_m = Km * 0.1; % Bridging torque = Km * i_NL, [tau_m] = Nm
U_max = 9; % Maximum voltage, [U] = V
tau_max = 0.1177; % Maximum wheel torque

theta_max = asind(N*eta*Km*U_max/(M*g*l*R));
param = [m; M; g; I; J; r; l; b; h; c; d; N; R; L; Km; eta];
