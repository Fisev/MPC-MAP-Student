function [pose, score] = scan_match_pose(read_only_vars, prior_pose)

    if nargin < 2
        prior_pose = [];
    end

    map = read_only_vars.map;
    lidar = read_only_vars.lidar_distances;
    cfg = read_only_vars.lidar_config;
    gnss = read_only_vars.gnss_position;
    discrete_map = read_only_vars.discrete_map.map;
    step = read_only_vars.map.discretization_step;
    map_limits = read_only_vars.discrete_map.limits;

    if ~isempty(prior_pose)
        x_grid = prior_pose(1) + (-0.3:0.15:0.3);
        y_grid = prior_pose(2) + (-0.3:0.15:0.3);
        theta_grid = prior_pose(3) + (-pi/6 : pi/24 : pi/6);
    elseif ~isnan(gnss(1))
        x_grid = gnss(1) + (-0.4:0.2:0.4);
        y_grid = gnss(2) + (-0.4:0.2:0.4);
        theta_grid = (-pi):(pi/8):(pi - 1e-6);  
    else
        margin = 0.6;
        x_grid = (map.limits(1) + margin):0.4:(map.limits(3) - margin);
        y_grid= (map.limits(2) + margin):0.4:(map.limits(4) - margin);
        theta_grid = (-pi):(pi/8):(pi - 1e-6);
    end

    [best_pose, best_score] = grid_search(map, lidar, cfg, x_grid, y_grid, theta_grid, discrete_map, step, map_limits);

    x_fine = best_pose(1) + (-0.15 : 0.05 : 0.15);
    y_fine = best_pose(2) + (-0.15 : 0.05 : 0.15);
    theta_fine = best_pose(3) + (-pi/12 : pi/64 : pi/12);

    [refined_pose, refined_score] = grid_search(map, lidar, cfg, x_fine, y_fine, theta_fine, discrete_map, step, map_limits);

    if refined_score <= best_score
        pose = refined_pose;
        score = refined_score;
    else
        pose = best_pose;
        score = best_score;
    end
end


function [best_pose, best_score] = grid_search(map, lidar, cfg, x_grid, y_grid, theta_grid, discrete_map, step, map_limits)
    best_score = inf;
    best_pose  = [x_grid(1), y_grid(1), theta_grid(1)];

    [ySize, xSize] = size(discrete_map);

    for x = x_grid
        cx = floor((x - map_limits(1)) / step) + 1;
        if cx < 1 || cx > xSize, continue; end
        for y = y_grid
            cy = floor((y - map_limits(2)) / step) + 1;
            if cy < 1 || cy > ySize, continue; end

            if discrete_map(cy, cx), continue; end
            for theta = theta_grid
                expected = compute_lidar_measurement(map, [x, y, theta], cfg);
                mask = isfinite(expected) & expected > 0.01 & isfinite(lidar)    & lidar    > 0.01;
                if sum(mask) < 3
                    continue;
                end
                diff = expected(mask) - lidar(mask);
                score = mean(diff.^2);
                if score < best_score
                    best_score = score;
                    best_pose  = [x, y, theta];
                end
            end
        end
    end
end
