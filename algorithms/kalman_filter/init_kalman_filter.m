function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf_enabled = 1;
public_vars.kf.use_known_initial_pose = 0; % 1 = Task 3, 0 = Task 4
public_vars.kf.init_samples = max(100, public_vars.init_iterations);
public_vars.init_iterations = public_vars.kf.init_samples;

public_vars.kf.C = [1, 0, 0; 0, 1, 0];
public_vars.kf.R = diag([0.01, 0.01, 0.01]); % process noise
public_vars.kf.Q = diag([0.25, 0.25]);        % updated after GNSS initialization
public_vars.kf.agent_drive = read_only_vars.agent_drive;

public_vars.kf.gnss_samples = [];
public_vars.kf.is_initialized = 0;

if public_vars.kf.use_known_initial_pose
    public_vars.mu = [2; 2; pi/2];
    public_vars.sigma = zeros(3,3);
    public_vars.kf.is_initialized = 1;
else
    if any(isnan(read_only_vars.gnss_position))
        public_vars.mu = [0; 0; 0];
    else
        public_vars.mu = [read_only_vars.gnss_position(1); read_only_vars.gnss_position(2); 0];
    end
    public_vars.sigma = diag([10, 10, pi^2]);
end

end
