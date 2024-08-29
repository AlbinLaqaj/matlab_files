% function parameters()

    % Define global parameters
    global m M g I J r l b t c d N R L Km eta param
    
    % %% Räder- und Chassis-Parameter
    
    r = 0.044; % Radius Rad, [r] = m
    m = 0.018 + 0.0095; % m_rad = 0,018 + m_d = 0,01, [m] = kg
    M = 0.3303 - 2*m; % Masse Chassis, [M] = kg
    l = 0.05; % Abstand Verbindugslinie und COM Chassis, [l] = m
    t = 0.03; % Dicke des Chassis, [t] = m
    b = 0.11; % Breite Chassis, [b] = m
    g = 9.81; % Erdbeschleunigung, [g] = m/s^2
    I = M/12*(t^2 + b^2) + M*l^2; % Massenträgheit Chassis, [I] = kg*m^2
    J = 1/2*m*r^2; % Massenträgheit Rad, [J] = kg*m^2
    d = 0.005; % Schätzung
    c = 0.005; % Schätzung
    
    % %% Motorparameter
    
    N = 50; % Übersetzungsverhältnis, [N] = 1
    R = 0.5; % Schätzung, Ankerwiderstand, [Ra] = Ohm
    L = 2.6e-3; % Ankerinduktivität, [La] = H
    Km = 0.05; % Schätzung, Motorkonstante, [Km] = V/rad/s 
    eta = 0.9; % Schätzung, Getriebewirkungsgrad (90%), [eta] = 1
    % tau_m = ;
    
    
    param = [m; M; g; I; J; r; l; b; t; c; d; N; R; L; Km; eta];

% end