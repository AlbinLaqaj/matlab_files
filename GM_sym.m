function GM = GM_sym(q, M, g, l)
    % Globale Parameter deklarieren
    % global M g l

    % Variablen aus q extrahieren
    theta = q(3);

    % Elemente der G(q)-Matrix berechnen
    GM = [0; 0; -M*g*l*sin(theta)];
end