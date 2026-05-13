function [public_vars] = plan_motion(read_only_vars, public_vars)

    pose= public_vars.estimated_pose;
    path = public_vars.path;

    if isempty(pose) || any(isnan(pose))
        public_vars.motion_vector = [0, 0];
        return;
    end

    if isempty(path) || size(path, 1) < 1
        public_vars.motion_vector = [0, 0];
        return;
    end

    x = pose(1);
    y = pose(2);
    theta =pose(3);

    if isfield(public_vars, 'use_pf') && public_vars.use_pf
        lookahead = 0.6;
        rot_thr = pi/4;
        w_max = 4.5;
    else
        lookahead = 1.0;   %lookahead
        rot_thr = pi/4;
        w_max = 3.5;
    end

    %I. Find the closest waypoint
    dx = path(:, 1) - x;
    dy = path(:, 2) - y;
    d  = hypot(dx, dy);
    [~, i_near] = min(d);

    target_idx = size(path, 1);
    for i = i_near:size(path, 1)
        if hypot(path(i, 1) - x, path(i, 2) - y) >= lookahead
            target_idx = i;
            break;
        end
    end
    target = path(target_idx, :);

    goal = path(end, :);
    dist_goal = hypot(goal(1) - x, goal(2) - y);

    target_heading = atan2(target(2) - y, target(1) - x);
    heading_error = atan2(sin(target_heading - theta), cos(target_heading - theta));

    %II. wall avoidance from lidlar
    lidar = read_only_vars.lidar_distances;
    front_dist = inf;
    for i = [1, 2, 8]
        if i <= length(lidar) && lidar(i) > 0 && lidar(i) < front_dist
            front_dist = lidar(i);
        end
    end

    safe_dist = 0.4;

    if dist_goal < 0.4
        crit_dist = 0.05;
    else
        crit_dist = 0.2;
    end

    if front_dist < crit_dist
        speed_scale = 0;
    elseif front_dist < safe_dist
        speed_scale = (front_dist - crit_dist) / (safe_dist - crit_dist);
    else
        speed_scale = 1;
    end

    %III. velocity calculation
    v_max = read_only_vars.agent_drive.max_vel;
    k_w = 2.5;

    if abs(heading_error) > rot_thr
        v = 0;
        w = sign(heading_error) * w_max;
    else
        v = v_max * max(cos(heading_error), 0.8);

        if dist_goal < 0.5
            v = min(v, max(dist_goal, 0.15));
        end

        v = v * speed_scale;
        w = k_w * heading_error;
        w = max(min(w, w_max), -w_max);
    end

    b  = read_only_vars.agent_drive.interwheel_dist;
    vR = v + 0.5 * b * w;
    vL = v - 0.5 * b * w;

    s = max(abs(vR), abs(vL));
    if s > v_max
        vR = vR * (v_max / s);
        vL = vL * (v_max / s);
    end

    public_vars.motion_vector = [vR, vL];
end
