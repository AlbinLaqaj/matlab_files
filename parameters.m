% Define global parameters
global m M g I J r l b t c d N R L Km eta param

%% Räder- und Chassis-Parameter

r = 0.044; % Außenradius Rad, [r] = m
m = 0.018 + 0.0095; % m_rad = 0,018 + m_dc = 0,0095, [m] = kg
M = 0.3303 - 2*m; % Masse Chassis, [M] = kg
l = 0.05; % Abstand Verbindugslinie und COM Chassis, [l] = m
t = 0.03; % Tiefe des Chassis, [t] = m
b = 0.105; % Breite Chassis, [b] = m
g = 9.81; % Erdbeschleunigung, [g] = m/s^2
I = M/12*(t^2 + b^2) + M*l^2; % Massenträgheit Chassis, [I] = kg*m^2
J = 1/2*m*r^2; % Massenträgheit Rad, [J] = kg*m^2
d = 0.005; % keine Begründung, Reibung zw Räder und Boden
c = 0.01; % Reibung zw Chassis und dc


%% Motorparameter

N = 50; % Übersetzungsverhältnis, [N] = 1
R = 12.3; % Ankerwiderstand, [Ra] = Ohm
L = 2.6e-3; % Ankerinduktivität, [La] = H
Km = 5.27e-3; % Motorkonstante, [Km] = V/rad/s 
eta = 0.3; % Getriebewirkungsgrad, [eta] = 1
tau_m = Km * 0.052; % Überbrückungsmoment = Km * i_NL, [tau_m] = Nm
U_max = 7.2; % max voltage, [U] = V
tau_max = 0.053; % max momentum of wheels


param = [m; M; g; I; J; r; l; b; t; c; d; N; R; L; Km; eta];