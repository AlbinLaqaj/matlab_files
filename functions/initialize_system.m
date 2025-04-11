function [init_q, init_dq, theta_ref] = initialize_system()

% Outputs:
%    init_q  - Vector containing initial positions [phi1; phi2; theta] in radians.
%    init_dq - Vector containing initial velocities [dphi1; dphi2; dtheta] in radians per second.

    % Initialize states
    init_phi1 = 0;
    init_phi2 = 0;
    init_theta = 15 * (pi/180);
    init_dphi1 = 0;
    init_dphi2 = 0;
    init_dtheta = 0;

    init_q = [init_phi1; init_phi2; init_theta];
    init_dq = [init_dphi1; init_dphi2; init_dtheta];

    theta_ref = 0;

end
