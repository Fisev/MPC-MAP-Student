function [path] = astar(read_only_vars, public_vars)
%ASTAR Summary of this function goes here

    map = read_only_vars.discrete_map.map;
    step= read_only_vars.map.discretization_step;
    limits = read_only_vars.discrete_map.limits;     % [xmin ymin xmax ymax]

    [ySize, xSize] = size(map);

    hard_kernel = ones(3);
    hard_inflated = conv2(double(map), hard_kernel, 'same') > 0;

    hard_inflated(1, :) = true;
    hard_inflated(end, :) = true;
    hard_inflated(:, 1) = true;
    hard_inflated(:, end) = true;

    soft_kernel = ones(7);
    soft_zone = (conv2(double(map), soft_kernel, 'same') > 0) & ~hard_inflated;

    cost_field = ones(ySize, xSize);
    cost_field(soft_zone) = 4; 

    startCellX = clamp(floor((public_vars.estimated_pose(1) - limits(1)) / step) + 1, 1, xSize);
    startCellY = clamp(floor((public_vars.estimated_pose(2) - limits(2)) / step) + 1, 1, ySize);
    goalCellX  = clamp(floor((read_only_vars.map.goal(1) - limits(1)) / step) + 1, 1, xSize);
    goalCellY  = clamp(floor((read_only_vars.map.goal(2)- limits(2)) / step) + 1, 1, ySize);

    hard_inflated = clear_cell(hard_inflated, startCellY, startCellX, ySize, xSize);
    hard_inflated = clear_cell(hard_inflated, goalCellY,  goalCellX,  ySize, xSize);

    start = [startCellY, startCellX];
    goal  = [goalCellY,  goalCellX];

    actions = [-1  0;  1  0;  0 -1;  0  1;
               -1 -1; -1  1;  1 -1;  1  1];
    move_costs = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];

    G = inf(ySize, xSize);
    parentR = zeros(ySize, xSize);
    parentC = zeros(ySize, xSize);
    closed  = false(ySize, xSize);

    G(start(1), start(2)) = 0;
    fStart = norm(start - goal) * step;

    queue = [fStart, start(1), start(2)];   % [f, r, c]
    found = false;

    while ~isempty(queue)
        [~, idx] = min(queue(:, 1));
        row = queue(idx, 2);
        col = queue(idx, 3);
        queue(idx, :) = [];

        if closed(row, col)
            continue;
        end
        closed(row, col) = true;

        if row == goal(1) && col == goal(2)
            found = true;
            break;
        end

        for k = 1:size(actions, 1)
            nr = row + actions(k, 1);
            nc = col + actions(k, 2);
            if nr < 1 || nr > ySize || nc < 1 || nc > xSize, continue; end
            if hard_inflated(nr, nc) || closed(nr, nc),      continue; end

            g_new = G(row, col) + move_costs(k) * step * cost_field(nr, nc);
            if g_new < G(nr, nc)
                G(nr, nc) = g_new;
                parentR(nr, nc) = row;
                parentC(nr, nc) = col;
                f_new = g_new + norm([nr, nc] - goal) * step;
                queue = [queue; f_new, nr, nc];
            end
        end
    end

    if ~found
        path = [public_vars.estimated_pose(1), public_vars.estimated_pose(2);
                read_only_vars.map.goal(1), read_only_vars.map.goal(2)];
        return;
    end

    cells = goal;
    row = goal(1); col = goal(2);
    while ~(row == start(1) && col == start(2))
        pr = parentR(row, col);
        pc = parentC(row, col);
        cells = [pr, pc; cells];
        row = pr;
        col = pc;
    end

    ys = limits(2) + (cells(:, 1) - 0.5) * step;
    xs = limits(1) + (cells(:, 2) - 0.5) * step;

    path = [xs, ys];
    path(end, :) = [read_only_vars.map.goal(1), read_only_vars.map.goal(2)];
end


function v = clamp(v, lo, hi)
    v = max(lo, min(hi, v));
end

function inflated = clear_cell(inflated, r, c, ySize, xSize)
    rs = max(1, r-1):min(ySize, r+1);
    cs = max(1, c-1):min(xSize, c+1);
    inflated(rs, cs) = false;
end
