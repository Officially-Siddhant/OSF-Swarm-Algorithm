function result = compute_damping_force(robot, target)
    K_DAMP = 0.8;
    K4 = 1.2;
    R = 1.2;

    % 2D position and velocity
    qi = robot.pose;   % [x; y]
    qt = target.pose;  % [x; y]
    pi = robot.vel;    % [vx; vy]

    % Distance to target
    dist = norm(qi - qt);

    % Damping force
    if dist < K4 * R
        f_damp = -K_DAMP * pi;
    else
        f_damp = [0; 0];
    end

    result = f_damp;  % directly return [x; y]
end