function f_alpha = compute_flocking_force(robot, neighbors)
    % Constants
    EPSILON = 0.1;
    D = 2.0;
    KC = 1.2;
    R = KC * D;
    R_SIGMA = (1 / EPSILON) * (sqrt(1 + EPSILON * (R^2)) - 1);
    H = 0.2;
    a = 10.0;
    b = 10.0;
    c = abs(a - b) / sqrt(4 * a * b);
    d_alpha = (1 / EPSILON) * (sqrt(1 + EPSILON * (D^2)) - 1);

    qi = robot.pose;
    pi = robot.vel;
    f_alpha = zeros(2, 1);

    for j = 1:length(neighbors)
        qj = neighbors(j).pose;
        pj = neighbors(j).vel;

        q_diff = qj - qi;
        norm_sigma = sigma_norm(q_diff, EPSILON);

        if norm_sigma >= R_SIGMA
            continue;
        end

        n_ij = q_diff / sqrt(1 + EPSILON * dot(q_diff, q_diff));
        a_ij = bump_function(norm_sigma / R_SIGMA, H);
        grad_term = phi_alpha(norm_sigma, R_SIGMA, d_alpha, H, a, b, c);
        consensus_term = a_ij * (pj - pi);

        f_alpha = f_alpha + grad_term * n_ij + consensus_term;
    end
end

function sigma = sigma_norm(z, EPSILON)
    sigma = (1.0 / EPSILON) * (sqrt(1 + EPSILON * dot(z, z)) - 1);
end

function rho = bump_function(z, H)
    if z >= 0 && z < H
        rho = 1.0;
    elseif z >= H && z < 1
        rho = 0.5 * (1 + cos(pi * (z - H) / (1 - H)));
    else
        rho = 0.0;
    end
end

function val = phi(z, a, b, c)
    val = 0.5 * ((a + b) * (z + c) / sqrt(1 + (z + c)^2) + (a - b));
end

function result = phi_alpha(z, R_SIGMA, d_alpha, H, a, b, c)
    if z >= R_SIGMA
        result = 0.0;
    else
        result = bump_function(z / R_SIGMA, H) * phi(z - d_alpha, a, b, c);
    end
end