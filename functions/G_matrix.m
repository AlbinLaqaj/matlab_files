function GM = G_matrix(q)
    % Globale Parameter deklarieren
    global M g l

    % Variablen aus q extrahieren
    theta = q(3);

    % Elemente der G(q)-Matrix berechnen
    GM = [0; 0; -M*g*l*sin(theta)];
end
