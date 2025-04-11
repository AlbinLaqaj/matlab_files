
%% Wheel and chassis parameters

r = 0.044; % Outer radius of the wheel, [r] = m
m = 0.018 + 0.011; % m_rad = 0.018 + m_dc = 0.011, [m] = kg
M = 0.3303 - 2*m; % Chassis mass, [M] = kg
l = 0.053; % Distance between the connection line and the chassis COM, [l] = m
h = 0.03; % Chassis depth, [h] = m
b = 0.105; % Chassis width, [b] = m
g = 9.81; % Gravitational acceleration, [g] = m/s^2
I = M/12*(h^2 + b^2) + M*l^2; % Moment of inertia of the chassis, [I] = kg*m^2
J = 1/2*m*r^2; % Moment of inertia of the wheel, [J] = kg*m^2
d = 0; % friction between wheels and ground
c = 0.0022; % Friction between chassis and axle

%% Motor parameters

N = 50;   % Gear ratio, [N] = 1
R = 12.3; % Armature resistance, [R] = Ohm
L = 2.6e-3; % Armature inductance, [L] = H
Km = 5.27e-3; % Motor constant, [Km] = V/rad/s
eta = 0.3; % Gear efficiency, [eta] = 1
tau_m = Km * 0.052; % Bridging torque = Km * i_NL, [tau_m] = Nm
U_max = 7.2; % Maximum voltage, [U] = V
tau_max = 0.053; % Maximum wheel torque

theta_max = asin(N*eta*Km*U_max/(M*g*l*R));
param = [m; M; g; I; J; r; l; b; h; c; d; N; R; L; Km; eta];
