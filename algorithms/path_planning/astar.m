function [path] = astar(read_only_vars, public_vars)

    path = [];

    map    = read_only_vars.discrete_map.map;          % 1 = obstacle, 0 = free
    step   = read_only_vars.map.discretization_step;
    limits = read_only_vars.discrete_map.limits;       % [xmin ymin xmax ymax]

    inflationSize = ceil(0.2 / step);
    kernel = ones(2*inflationSize + 1);
    inflatedMap = conv2(double(map), kernel, 'same') > 0;

    [ySize, xSize] = size(map);    % rows = y, cols = x

    startCellX = floor((public_vars.estimated_pose(1)) / step) + 1;
    startCellY = floor((public_vars.estimated_pose(2)) / step) + 1;
    goalCellX  = floor((read_only_vars.map.goal(1) ) / step) + 1;
    goalCellY  = floor((read_only_vars.map.goal(2)) / step) + 1;

    start = [startCellY, startCellX];
    goal  = [goalCellY,  goalCellX];

    actions = [-1  0;  1  0;  0 -1;  0  1;
               -1 -1; -1  1;  1 -1;  1  1];
    costs   = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];

    G = inf(ySize, xSize);
    parentRow = zeros(ySize, xSize);
    parentCollumn = zeros(ySize, xSize);
    closed   = false(ySize, xSize);

    G(start(1), start(2)) = 0;
    fStart = norm(start - goal) * step;

    queue = [fStart, start(1), start(2)];      % [f, r, c]

    while ~isempty(queue)
        [~, idx] = min(queue(:, 1));
        row = queue(idx, 2);
        collumn = queue(idx, 3);
        queue(idx, :) = [];

        if closed(row, collumn)
            continue;
        end

        closed(row, collumn) = true;

        for k = 1:size(actions, 1)
            nr = row + actions(k, 1);
            nc = collumn + actions(k, 2);
            if nr < 1 || nr > ySize || nc < 1 || nc > xSize
                continue;
            end
            if inflatedMap(nr, nc) || closed(nr, nc)
                continue;
            end

            g_new = G(row, collumn) + costs(k) * step;
            if g_new < G(nr, nc)
                G(nr, nc) = g_new;
                parentRow(nr, nc) = row;
                parentCollumn(nr, nc) = collumn;
                f_new = g_new + norm([nr, nc] - goal) * step;
                queue = [queue; f_new, nr, nc];
            end
        end
    end

    cells = goal;
    row = goal(1); collumn = goal(2);
    while ~(row == start(1) && collumn == start(2))
        pr = parentRow(row, collumn);
        pc = parentCollumn(row, collumn);
        cells = [pr, pc; cells];
        row = pr; 
        collumn = pc;
    end

    ys = limits(2) + (cells(:, 1) - 0.5) * step;
    xs = limits(1) + (cells(:, 2) - 0.5) * step;
    path = [xs, ys];
end
