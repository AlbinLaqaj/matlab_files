function [A, B, C, D] = MinsegLinearization()

    global param

    % Initialization of symbolic variables
    syms q1 q2 q3 dq1 dq2 dq3
    syms tau1 tau2
    syms m M g I J r l b t c d
    
    % States and inputs
    q = [q1; q2; q3];
    dq = [dq1; dq2; dq3];
    x = [q; dq];
    tau = [tau1; tau2; 0];
    u = [tau1; tau2];
    
    % Symbolic parameters
    sym_chassis_param = [m; M; g; I; J; r; l; b; t; c; d];
    
    % System equations
    D_matrix = [d*r 0 0; 0 d*r 0; 0 0 c];

    dx(1:3) = dq;
    dx(4:6) = MM_sym(q, r, m, M, l, I, J) \ (tau - CM_sym(x, r, M, l) * dq - D_matrix*dq - GM_sym(q, M, g, l));
    
    % Calculation of Jacobian matrices
    pdf_pdx = jacobian(dx, x);
    pdf_pdu = jacobian(dx, u);
    
    % Equations for the systems
    eq_x = zeros(6, 1);
    eq_u = zeros(2, 1);
    
    % Symbolic matrices A and B
    A_sym = subs(pdf_pdx, [x; u], [eq_x; eq_u]);
    B_sym = subs(pdf_pdu, [x; u], [eq_x; eq_u]);
    
    % Substitute numerical values
    A = double(subs(A_sym, sym_chassis_param, param(1:11)));
    B = double(subs(B_sym, sym_chassis_param, param(1:11)));
    C = eye(6);
    D = zeros(6,2);
end


% help function for computing M_Matrix
function MM = MM_sym(q, r, m, M, l, I, J)

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


% help function for computing C_Matrix
function CM = CM_sym(x, r, M, l)

    % Variablen aus in = [q q_dot] extrahieren
    theta = x(3);
    theta_dot = x(6);

    % Elemente der C(q, q_dot)-Matrix berechnen
    CM(1,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(2,3) = -1/2*M*l*r*theta_dot*sin(theta);
    CM(3,3) = -M*l*r*theta_dot*sin(theta);
end


% help function for computing G_Matrix
function GM = GM_sym(q, M, g, l)

    % Variablen aus q extrahieren
    theta = q(3);

    % Elemente der G(q)-Matrix berechnen
    GM = [0; 0; -M*g*l*sin(theta)];
end