# Robotics-Tool
Robotics tool for some basic computing, like Kinematics, Dynamics and Gradients.
`
% Define the polygon obstacle
polygon = [0, 0; 1, 0; 1, 1; 0, 1];  % Square polygon obstacle

% Define the starting point and goal
start = [-0.5, 0.5];
goal = [1.5, 0.5];

% Set up the optimization problem
objective = @(p) norm(p - goal);  % Minimize distance to goal
nonlcon = @(p) sdf_constraint(p, polygon);  % SDF constraint (must be >= 0)

% Initial guess for optimization (start point)
p0 = start;

% Use fmincon to solve the optimization problem
options = optimoptions('fmincon', 'Display', 'iter');
[p_opt, fval] = fmincon(objective, p0, [], [], [], [], [], [], nonlcon, options);

% Plot the results
figure;
hold on;
fill(polygon(:, 1), polygon(:, 2), 'r', 'FaceAlpha', 0.3);  % Plot obstacle
plot(start(1), start(2), 'bo', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Goal');
plot(p_opt(1), p_opt(2), 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Optimal');
legend;
title('Trajectory Optimization with SDF Constraints');
xlabel('x');
ylabel('y');
axis equal;
grid on;

% Helper function: Compute the SDF constraint (SDF should be >= 0)
function [c, ceq] = sdf_constraint(p, polygon)
    % c represents inequality constraints (must be <= 0)
    % ceq represents equality constraints (none in this case)
    
    % Compute the signed distance function (SDF) at the point p
    d = compute_signed_distance(p(1), p(2), polygon);
    
    % SDF should be >= 0 (to avoid obstacle)
    c = -d;  % Negative SDF implies constraint violation (c <= 0)
    ceq = [];
end

% Helper function: Compute the signed distance function for a polygon
function d = compute_signed_distance(x, y, polygon)
    % Initialize the distance as a large number
    d = inf;
    
    % Loop over each edge of the polygon
    for i = 1:size(polygon, 1)
        % Get the current edge (from p1 to p2)
        p1 = polygon(i, :);
        p2 = polygon(mod(i, size(polygon, 1)) + 1, :);
        
        % Compute the distance to this edge
        edgeDistance = point_to_line_segment_distance(x, y, p1, p2);
        
        % Keep track of the minimum distance
        d = min(d, edgeDistance);
    end
    
    % Check if the point (x, y) is inside the polygon
    inside = inpolygon(x, y, polygon(:, 1), polygon(:, 2));
    
    % If inside, negate the distance to make it a signed distance
    if inside
        d = -d;
    end
end

% Helper function: Compute the distance from a point to a line segment
function d = point_to_line_segment_distance(x, y, p1, p2)
    v = p2 - p1;
    w = [x - p1(1), y - p1(2)];
    
    % Projection of w onto v
    c1 = w(1) * v(1) + w(2) * v(2);
    c2 = v(1)^2 + v(2)^2;
    
    % Clamp the projection value between 0 and 1
    t = max(0, min(1, c1 / c2));
    
    % Compute the closest point on the segment
    closest_point = p1 + t * v;
    
    % Compute the distance from (x, y) to the closest point
    d = sqrt((closest_point(1) - x)^2 + (closest_point(2) - y)^2);
end

`
