function tau2 = dc_motor2(U2, phi2_dot)
    % Motorparameter
    global Km R

    % Motormomente berechnen
    tau2 = (Km * U2 / R) - (Km^2 / R) * phi2_dot;
end