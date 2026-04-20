function [public_vars] = student_workspace(read_only_vars,public_vars)
    %STUDENT_WORKSPACE Summary of this function goes here
    % 100 samples provide a stable GNSS mean/covariance estimate for startup.
    GNSS_INIT_ITERATIONS = 100;

    if ~isfield(public_vars, 'init_iterations') || read_only_vars.counter == 1
        % Keep robot static for 100 samples to estimate initial GNSS statistics.
        public_vars.init_iterations = GNSS_INIT_ITERATIONS;
    end
    
    % 8. Perform initialization procedure
    if (read_only_vars.counter <= public_vars.init_iterations)
              
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars = init_kalman_filter(read_only_vars, public_vars);
    
    end
    
    % 9. Update particle filter
    public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    
    % 10. Update Kalman filter
    [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
    
    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
    
    % 12. Path planning
    public_vars.path = plan_path(read_only_vars, public_vars);
    
    % 13. Plan next motion command
    public_vars = plan_motion(read_only_vars, public_vars);
end
