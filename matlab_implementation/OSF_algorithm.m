close all;
clear; clc;

% Parameters
num_agents = 5;
dt = 0.01;      % timestep (20 Hz)
steps = 1000;   
is_informed = [true, false, false, true, false];

% Initializations
robots = repmat(struct('pose', zeros(2,1), 'vel', zeros(2,1)), num_agents, 1);
for i = 1:num_agents
    robots(i).pose = [(i - 1) * 2; 0];  % x = 0, 2, 4, 6, 8; y = 0
end

target.pose = [5; 5];
target.vel = [0; 0];
initial_poses = zeros(2, num_agents);

% Store trajectories
trajectory = zeros(2, steps, num_agents);

% Create figure and hold
figure;
axis equal; grid on;
xlim([-1, 10]); ylim([-1, 10]);
hold on;

% Initialize plot handles
robot_paths = gobjects(1, num_agents);
robot_dots = gobjects(1, num_agents);

for i = 1:num_agents
    robot_paths(i) = plot(NaN, NaN, 'LineWidth', 1.5);
    robot_dots(i) = plot(NaN, NaN, 'o', 'MarkerSize', 8);
end
target_marker = plot(target.pose(1), target.pose(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

% Simulation loop
for t = 1:steps
    for i = 1:num_agents
        neighbors = robots([1:i-1, i+1:end]);  % all except current

        % Flocking force
        f_alpha = compute_flocking_force(robots(i), neighbors);
        if i == 2
            disp(f_alpha)
        end
        % Navigation + damping if informed
        if is_informed(i) && ~isempty(target)
            f_nav = compute_navigation_force(robots(i), target, initial_poses(:,i));
            f_damp = compute_damping_force(robots(i), target);
        else
            f_nav = [0;0];
            f_damp = [0;0];
        end

        % Control force (OSF sum)
        f_total = f_alpha + f_nav + f_damp;

        % Update velocity and pose
        robots(i).vel = robots(i).vel + dt * f_total;
        robots(i).pose = robots(i).pose + dt * robots(i).vel;

        % Save trajectory
        trajectory(:, t, i) = robots(i).pose;

        % Update plots
        set(robot_paths(i), 'XData', trajectory(1,1:t,i), 'YData', trajectory(2,1:t,i));
        set(robot_dots(i), 'XData', robots(i).pose(1), 'YData', robots(i).pose(2));
    end
    
    drawnow;
end