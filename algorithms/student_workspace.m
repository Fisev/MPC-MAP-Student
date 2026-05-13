function [public_vars] = student_workspace(read_only_vars, public_vars)

    if read_only_vars.counter == 1
        if isnan(read_only_vars.gnss_position(1))
            public_vars.use_pf = true;
            public_vars.pf_tight_mode = false; 
            public_vars = init_particle_filter(read_only_vars, public_vars);
        else
            public_vars.use_pf = false;
            public_vars.pf_tight_mode = false;
        end
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.path = [];
        public_vars.pf_converged= false;
        public_vars.localize_countdown = 60; 
        public_vars.best_dist_to_goal = Inf;
        public_vars.stagnation_count = 0;
        public_vars.stuck_count = 0;
        public_vars.motion_vector = [0, 0];
        public_vars.init_iterations = 0;
    end

    % GNSS / PF switching

    gnss_now = ~isnan(read_only_vars.gnss_position(1));

    if ~gnss_now && ~public_vars.use_pf % entering indoor activate PF
        
        public_vars.use_pf = true;
        public_vars.pf_converged = true;   % skip exploration entirely
        public_vars.best_dist_to_goal = Inf;
        public_vars.stagnation_count = 0;
        public_vars.pose_window = [];

        N = read_only_vars.max_particles;
        xmin = read_only_vars.map.limits(1); xmax = read_only_vars.map.limits(3);
        ymin = read_only_vars.map.limits(2); ymax = read_only_vars.map.limits(4);
        cx = public_vars.mu(1);
        cy = public_vars.mu(2);
        ca = public_vars.mu(3);
        public_vars.particles = zeros(N, 3);
        public_vars.particles(:, 1) = cx + 0.4 * randn(N, 1);
        public_vars.particles(:, 2) = cy + 0.4 * randn(N, 1);
        public_vars.particles(:, 3) = ca + 0.3  * randn(N, 1);
        public_vars.particles(:, 1) = max(xmin, min(xmax, public_vars.particles(:, 1)));
        public_vars.particles(:, 2) = max(ymin, min(ymax, public_vars.particles(:, 2)));
        public_vars.pf_tight_mode = true;

    elseif gnss_now && public_vars.use_pf
        if ~any(isnan(public_vars.estimated_pose))
            public_vars.mu = public_vars.estimated_pose(:);
            public_vars.sigma = diag([0.5, 0.5, 0.2]);
        end
        public_vars.use_pf = false;
        public_vars.pf_converged = false;
        public_vars.path = [];
        public_vars.pose_window  = [];
        if isfield(public_vars, 'particle_pose')
            public_vars.particle_pose = [];
        end
    end

    if public_vars.use_pf
        public_vars = update_particle_filter(read_only_vars, public_vars);
    end
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

    % estimated pose smoothing
    raw_pose = estimate_pose(public_vars, read_only_vars);

    if public_vars.use_pf && public_vars.pf_converged && ~any(isnan(raw_pose))
        if ~isfield(public_vars, 'pose_window') || isempty(public_vars.pose_window)
            public_vars.pose_window = repmat(raw_pose, 3, 1);
        else
            public_vars.pose_window = [public_vars.pose_window(2:end, :); raw_pose];
        end
        sx = mean(public_vars.pose_window(:, 1));
        sy = mean(public_vars.pose_window(:, 2));
        sa = atan2(mean(sin(public_vars.pose_window(:, 3))), mean(cos(public_vars.pose_window(:, 3))));
        public_vars.estimated_pose = [sx, sy, sa];
    else
        public_vars.pose_window = [];
        public_vars.estimated_pose = raw_pose;
    end

    %Exploration usingPF

    if public_vars.use_pf && ~public_vars.pf_converged

        iters_done = 60 - public_vars.localize_countdown;

        if iters_done >= 20 && isfield(public_vars, 'pf_neff') && public_vars.pf_neff < (read_only_vars.max_particles / 10)
            public_vars.pf_converged = true;
        end

        if ~public_vars.pf_converged
            if public_vars.localize_countdown > 0
                public_vars.localize_countdown = public_vars.localize_countdown - 1;

                lidar_e = read_only_vars.lidar_distances;
                front_e = inf;
                for ei = [1, 2, 8]
                    if ei <= length(lidar_e) && lidar_e(ei) > 0
                        front_e = min(front_e, lidar_e(ei));
                    end
                end
                left_e  = lidar_e(7);  % 270 deg = left
                right_e = lidar_e(3);  % 90  deg = right

                side_e = min(left_e, right_e);

                if front_e < 0.4
                    if left_e >= right_e
                        public_vars.motion_vector = [-0.2, 0.5]; % turn left
                    else
                        public_vars.motion_vector = [0.5, -0.2]; % turn right
                    end
                elseif side_e < 0.35
                    %Narrow slow down
                    public_vars.motion_vector = [0.4, 0.4];
                else
                    % Open go fast
                    public_vars.motion_vector = [0.8, 0.8];
                end

                public_vars.path = [];
                return;
            else
                public_vars.pf_converged = true;
            end
        end
    end

    if public_vars.use_pf && ~any(isnan(public_vars.estimated_pose))
        curr_dist = norm(public_vars.estimated_pose(1:2) - read_only_vars.map.goal(1:2));

        if curr_dist < public_vars.best_dist_to_goal - 0.3
            public_vars.best_dist_to_goal = curr_dist;
            public_vars.stagnation_count  = 0;
        else
            public_vars.stagnation_count = public_vars.stagnation_count + 1;
        end

        if public_vars.stagnation_count > 200
            public_vars = init_particle_filter(read_only_vars, public_vars);
            public_vars.path = [];
            public_vars.pf_converged= false;
            public_vars.localize_countdown = 60;
            public_vars.best_dist_to_goal  = Inf;
            public_vars.stagnation_count= 0;
            public_vars.pose_window = [];  
            public_vars.pf_tight_mode = false; 
            public_vars.motion_vector = [0.8, 0.8];
            return;
        end
    end

    % Navigation.

    lidar = read_only_vars.lidar_distances;

    front_blocked = false;
    for i = [1, 2, 8]
        if i <= length(lidar) && lidar(i) > 0 && lidar(i) < 0.30
            front_blocked = true;
            break;
        end
    end

    if ~isempty(public_vars.path)
        if front_blocked
            public_vars.path = [];
        elseif ~public_vars.use_pf
            dx = public_vars.path(:, 1) - public_vars.estimated_pose(1);
            dy = public_vars.path(:, 2) - public_vars.estimated_pose(2);
            if min(hypot(dx, dy)) > 1.5
                public_vars.path = [];
            end
        end
    end

    % Path planning A* cached
    public_vars.path = plan_path(read_only_vars, public_vars);

    % motion control pure pursuit and wall avoidance
    public_vars = plan_motion(read_only_vars, public_vars);

    if ~public_vars.use_pf && public_vars.init_iterations < 25
        public_vars.init_iterations = public_vars.init_iterations + 1;
        vR = public_vars.motion_vector(1);
        vL = public_vars.motion_vector(2);
        if public_vars.init_iterations <= 15
            % pure rotation, zero translation.
            rot = (vR - vL) / 2;
            public_vars.motion_vector = [rot, -rot];
        else
            ramp = (public_vars.init_iterations - 15) / 10;
            public_vars.motion_vector = public_vars.motion_vector * max(ramp, 0.3);
        end
    end

    % recovery
    if max(abs(public_vars.motion_vector)) < 0.05
        public_vars.stuck_count = public_vars.stuck_count + 1;
    else
        public_vars.stuck_count = 0;
    end

    if public_vars.stuck_count > 5
        if min([lidar(1), lidar(2), lidar(8)]) > 0.25
            public_vars.motion_vector = [0.4, 0.4];   % nudge forward
        else
            public_vars.motion_vector = [-0.4, -0.4]; % back up
        end
        public_vars.path = [];
        public_vars.stuck_count = 0;
    end
end
