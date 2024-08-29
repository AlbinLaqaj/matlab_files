function MM = MM_sym(q, r, m, M, l, I, J)
    % Globale Parameter deklarieren
    % global r m M l I J

    % Variablen aus q extrahieren
    theta = q(3);

    % Elemente der M(q)-Matrix berechnen
    MM(1,1) = r^2*(1/4*M + m) + J;
    MM(1,2) = 1/4*M*r^2;
    MM(1,3) = r^2*(1/2*M + m) + J + 1/2*M*l*r*cos(theta);
    MM(2,1) = 1/4*M*r^2;
    MM(2,2) = r^2*(1/4*M + m) + J;
    MM(2,3) = r^2*(1/2*M + m) + J + 1/2*M*l*r*cos(theta);
    MM(3,1) = r^2*(1/2*M + m) + J + 1/2*M*l*r*cos(theta);
    MM(3,2) = r^2*(1/2*M + m) + J + 1/2*M*l*r*cos(theta);
    MM(3,3) = r^2*(M + 2*m) + 2*J + 2*M*l*r*cos(theta) + M*l^2 + I;
end
