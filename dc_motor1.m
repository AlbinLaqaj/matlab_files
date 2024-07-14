function tau1 = dc_motor1(U1, phi1_dot)
    % Motorparameter
    global Km R

    % Motormomente berechnen
    tau1 = (Km * U1 / R) - (Km^2 / R) * phi1_dot;
end