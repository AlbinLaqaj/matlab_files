%%% Define global parameters
global r m M l g I J b c d

%% Räder- und Chassis-Parameter

r = 0.044; % Messwert, Radius Rad, [r] = m,
m = 0.028; % Messwert, m_rad = 0,02 + m_d = 0,01, [m] = kg,
M = 0.3303 - 2*m; % Messwert, Masse Chassis, [M] = kg,
l = 0.06; % Schätzung, Abstand Verbindugslinie und COM Chassis, [l] = m,
t = 0.03; % Messwert, Dicke des Chassis, [t] = m
b = 0.11; % Messwert, Breite Chassis, [b] = m
g = 9.81; % Erdbeschleunigung, [g] = m/s^2
I = M/12*(t^2 + b^2) + M*l^2; % Massenträgheit Chassis, [I] = kg*m^2
J = 1/2*m*r^2; % Massenträgheit Rad, [J] = kg*m^2
d = 0.005; % Schätzung
c = 0.005; % Schätzung

%% Input

U1 = 0; % Motorspannung, [U1] = V
U2 = 0; % Motorspannung, [U2] = V

%% Motorparameter
N = 50; % Schätzung, Übersetzungsverhältnis, [N] = 1
R = 0.5; % Schätzung, Ankerwiderstand, [Ra] = Ohm
L = 2.6e-3; % Messwert, Ankerinduktivität, [La] = H
Km = 0.05; % Schätzung, Motorkonstante, [Km] = V/rad/s 
eta = 0.9; % Schätzung, Getriebewirkungsgrad (90%), [eta] = 1

tau1 = 0;
tau2 = 0;

