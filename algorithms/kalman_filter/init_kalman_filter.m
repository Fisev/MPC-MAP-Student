function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Collect GNSS samples to estimate measurement noise,

    gnssInitSamples = 50;

    if ~isfield(public_vars, 'kf')
        public_vars.kf.gnss_samples = zeros(0, 2);
        public_vars.kf.C = [1, 0, 0;
                            0, 1, 0];                               % observation matrix
        public_vars.kf.interwheel_dist = read_only_vars.agent_drive.interwheel_dist;
    end

    public_vars.kf.gnss_samples(end+1, :) = read_only_vars.gnss_position(1:2).';
    n = size(public_vars.kf.gnss_samples, 1);

    public_vars.mu    = [2; 2; pi/2];
    public_vars.sigma = zeros(3, 3);
    
    public_vars.kf.gnss_samples(end+1, :) = read_only_vars.gnss_position(1:2).';

    if n >= gnssInitSamples
        gnssMean = mean(public_vars.kf.gnss_samples, 1);
        gnssCov  = cov(public_vars.kf.gnss_samples);            

        public_vars.sigma = blkdiag(gnssCov, pi^2);
        public_vars.mu = [gnssMean(1); gnssMean(2); pi/2];
        
        public_vars.kf.Q = gnssCov;                                                 % measurement noise of sensor
        public_vars.kf.R = diag([0.0001, 0.0001, 0.0001]);                           % process noise of - noise of model
        public_vars.init_iterations = 0;
    end
    %public_vars.mu    = [2; 2; pi/2];
    %public_vars.sigma = zeros(3, 3);
end
