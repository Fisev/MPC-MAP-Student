function [path] = astar(read_only_vars, public_vars)
%ASTAR Plans a path on the occupancy grid using A* with a priority queue.
%   8-connected grid, Euclidean heuristic. Obstacles are inflated so that
%   the returned path keeps a clearance of at least 0.2 m.

    path = [];

    % I. Retrieve map and parameters
    map    = read_only_vars.discrete_map.map;          % 1 = obstacle, 0 = free
    step   = read_only_vars.map.discretization_step;
    limits = read_only_vars.discrete_map.limits;       % [xmin ymin xmax ymax]

    % II. Inflate obstacles to enforce 0.2 m clearance
    inflationSize = ceil(0.2 / step);
    kernel = ones(2*inflationSize + 1);
    inflatedMap = conv2(double(map), kernel, 'same') > 0;

    [ySize, xSize] = size(map);    % rows = y, cols = x

    % III. Convert continuous start/goal to grid cells [row, col] = [yCell, xCell]
    startCellX = floor((public_vars.estimated_pose(1) - limits(1)) / step) + 1;
    startCellY = floor((public_vars.estimated_pose(2) - limits(2)) / step) + 1;
    goalCellX  = floor((read_only_vars.map.goal(1)    - limits(1)) / step) + 1;
    goalCellY  = floor((read_only_vars.map.goal(2)    - limits(2)) / step) + 1;

    start = [startCellY, startCellX];
    goal  = [goalCellY,  goalCellX];

    if inflatedMap(start(1), start(2)) || inflatedMap(goal(1), goal(2))
        disp('A*: start or goal is occupied.');
        return;
    end

    actions = [-1  0;  1  0;  0 -1;  0  1;
               -1 -1; -1  1;  1 -1;  1  1];
    costs   = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];

    G = inf(ySize, xSize);
    parent_r = zeros(ySize, xSize);
    parent_c = zeros(ySize, xSize);
    closed   = false(ySize, xSize);

    G(start(1), start(2)) = 0;
    f_start = norm(start - goal) * step;

    queue = [f_start, start(1), start(2)];      % [f, r, c]

    % VI. A* main loop
    found = false;
    while ~isempty(queue)
        % Pop the node with the lowest f
        [~, idx] = min(queue(:, 1));
        r = queue(idx, 2);
        c = queue(idx, 3);
        queue(idx, :) = [];

        if closed(r, c)
            continue;
        end

        if r == goal(1) && c == goal(2)
            found = true;
            break;
        end

        closed(r, c) = true;

        for k = 1:size(actions, 1)
            nr = r + actions(k, 1);
            nc = c + actions(k, 2);
            if nr < 1 || nr > ySize || nc < 1 || nc > xSize
                continue;
            end
            if inflatedMap(nr, nc) || closed(nr, nc)
                continue;
            end

            g_new = G(r, c) + costs(k) * step;
            if g_new < G(nr, nc)
                G(nr, nc) = g_new;
                parent_r(nr, nc) = r;
                parent_c(nr, nc) = c;
                f_new = g_new + norm([nr, nc] - goal) * step;
                queue = [queue; f_new, nr, nc];
            end
        end
    end

    cells = goal;
    r = goal(1); c = goal(2);
    while ~(r == start(1) && c == start(2))
        pr = parent_r(r, c);
        pc = parent_c(r, c);
        cells = [pr, pc; cells];
        r = pr; c = pc;
    end

    ys = limits(2) + (cells(:, 1) - 0.5) * step;
    xs = limits(1) + (cells(:, 2) - 0.5) * step;
    path = [xs, ys];
end
