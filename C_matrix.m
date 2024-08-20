function CM = C_matrix(in)
    % Globale Parameter deklarieren
    global r M l

    % Variablen aus in = [q q_dot] extrahieren
    theta = in(3);
    theta_dot = in(6);

    % Initialisiere eine 3x3-Matrix
    CM = zeros(3,3); 

    % Elemente der C(q, q_dot)-Matrix berechnen
    CM(1,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(2,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(3,3) = -M*l*r*theta_dot*sin(theta);
end