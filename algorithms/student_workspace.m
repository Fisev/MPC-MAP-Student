function [public_vars] = student_workspace(read_only_vars,public_vars)
    %STUDENT_WORKSPACE Summary of this function goes here
    
    % 8. Perform initialization procedure
    if (read_only_vars.counter == 1)
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        
        if public_vars.pf_enabled
            public_vars = init_particle_filter(read_only_vars, public_vars);
        end
    
    end
    
    % 9. Update particle filter
    if public_vars.pf_enabled
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    end
    
    % 10. Update Kalman filter
    if public_vars.kf_enabled
        [public_vars.mu, public_vars.sigma, public_vars.kf] = update_kalman_filter(read_only_vars, public_vars);
    end
    
    % 11. Estimate current robot position
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
    
    % 12. Path planning
    public_vars.path = plan_path(read_only_vars, public_vars);
    
    % 13. Plan next motion command
    public_vars = plan_motion(read_only_vars, public_vars);
end
