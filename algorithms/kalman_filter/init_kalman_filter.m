function [public_vars] = init_kalman_filter(read_only_vars, public_vars)

    if ~isfield(public_vars, 'kf')
        public_vars.kf.C = [1, 0, 0; 0, 1, 0];
        public_vars.kf.interwheel_dist = read_only_vars.agent_drive.interwheel_dist;
        public_vars.kf.Q = diag([0.1, 0.1]);
        public_vars.kf.R = diag([1e-4, 1e-4, 1e-4]);
    end

    if isfield(public_vars, 'particle_pose') && ~isempty(public_vars.particle_pose)
        public_vars.mu = public_vars.particle_pose(:);
    else
        cx = (read_only_vars.map.limits(1) + read_only_vars.map.limits(3)) / 2;
        cy = (read_only_vars.map.limits(2) + read_only_vars.map.limits(4)) / 2;
        public_vars.mu = [cx; cy; 0];
    end
    public_vars.sigma= diag([1, 1, 1]);
    public_vars.init_iterations = 0;
end
