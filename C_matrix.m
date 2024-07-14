function CM = C_matrix(q, q_dot)
    % Globale Parameter deklarieren
    global r M l

    % Variablen aus q und q_dot extrahieren
    theta = q(3);
    theta_dot = q_dot(3); %fdgsif

    % Elemente der C(q, q_dot)-Matrix berechnen
    CM = zeros(3,3); % Initialisiere eine 3x3-Matrix
    CM(:,3) = -(1/2)*M*l*r*theta_dot*sin(theta) * [1; 1; 0];
end