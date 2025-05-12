function result = compute_navigation_force(robot, target, initial_pose_i)
    % Constants
    K1 = 0.9;
    K2 = 0.5;
    K3 = 0.5;
    c1 = 0.8;
    c2 = 0.5;

    % Extract current state
    qi = robot.pose;   % [x; y]
    pi = robot.vel;    % [vx; vy]
    qt = target.pose;  % [x; y]
    pt = target.vel;   % [vx; vy]

    % Compute distances
    dist_now = norm(qi - qt);
    dist_initial = norm(initial_pose_i - qt);

    % Apply control logic
    if dist_now > K1 * dist_initial
        % Far: normalized attraction
        direction = qi - qt;
        norm_dir = norm(direction) + 1e-6; % avoid divide-by-zero
        pos_term = -K2 * direction / norm_dir;
        vel_term = -K3 * (pi - pt) / norm_dir;
        f_t = pos_term + vel_term;
    else
        % Close: standard linear gains
        f_t = -c1 * (qi - qt) - c2 * (pi - pt);
    end
    
    % Return Vector3-style struct
    result = f_t;  % directly return [x; y];  % 2D sim => z = 0
end