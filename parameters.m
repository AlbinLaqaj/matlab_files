% Define global parameters
global r m M l t g d c Km R I J

% Räder- und Chassis-Parameter
r = 0.044; % Radius Rad, [r] = m
m = 0.03; % m_rad = 0,02 + m_d = 0,01, [m] = kg
M = 0.5 - m; % Masse Chassis, [M] = kg
l = 0.06; % Abstand Verbindugslinie und COM Chassis, [l] = m
t = 0.03; % Dicke des Chassis, [t] = m
b = 0.11; % Breite Chassis, [b] = m
g = 9.81; % Erdbeschleunigung, [g] = m/s^2
d = 0.05; % Viskoser Dämpfungskoeffizient Räder/Tisch (glatt), [d] = N·s/m
c = 0.05; % Dämpfung zwischen Chassis, [c] = N·m·s/rad
I = M/12*(t^2+b^2) + M*l^2; % Massenträgheit Chassis, [I] = kg*m^2
J = 1/2*m*r^2; % Massenträgheit Rad, [J] = kg*m^2

% Motorparameter
U1 = 0; % Motorspannung, [U1] = V
U2 = 0; % Motorspannung, [U2] = V
N = 50; % Übersetzungsverhältnis, [N] = 1
R = 0.5; % Ankerwiderstand, [Ra] = Ohm
L = 2.6e-3; % Ankerinduktivität, [La] = H
Km = 0.05; % Motorkonstante, [Km] = V/rad/s 
eta = 0.9; % Getriebewirkungsgrad (60%), [eta] = 1

