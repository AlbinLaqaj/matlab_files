function MM = M_matrix(q)
    % Globale Parameter deklarieren
    global r m M l I J

    % Variablen aus q extrahieren
    phi1 = q(1);
    phi2 = q(2);
    theta = q(3);

    % Elemente der M(q)-Matrix berechnen
    MM = zeros(3,3); % Initialisiere eine 3x3-Matrix
    MM(1,1) = m*r^2 + J + (1/4)*M*r^2;
    MM(1,2) = (1/4)*M*r^2;
    MM(1,3) = (1/2)*M*l*cos(theta);
    MM(2,1) = (1/4)*M*r^2;
    MM(2,2) = m*r^2 + J + (1/4)*M*r^2;
    MM(2,3) = (1/2)*M*l*cos(theta);
    MM(3,1) = (1/2)*M*l*cos(theta);
    MM(3,2) = (1/2)*M*l*cos(theta);
    MM(3,3) = M*l^2+I;
end