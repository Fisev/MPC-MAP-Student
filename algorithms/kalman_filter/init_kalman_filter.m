function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

init_samples = 100;

if ~isfield(public_vars, "init_iterations") || public_vars.init_iterations < init_samples
    public_vars.init_iterations = init_samples;
end

if ~isfield(public_vars, "kf") || ~isfield(public_vars.kf, "gnss_samples")
    public_vars.kf.gnss_samples = zeros(0,2);
end

gnss = read_only_vars.gnss_position(1:2);
if all(isfinite(gnss))
    public_vars.kf.gnss_samples = [public_vars.kf.gnss_samples; gnss];
end

public_vars.kf.C = eye(2);

if size(public_vars.kf.gnss_samples, 1) >= 2
    gnss_mean = mean(public_vars.kf.gnss_samples, 1)';
    gnss_cov = cov(public_vars.kf.gnss_samples);
else
    gnss_mean = gnss(:);
    gnss_cov = eye(2) * 1e-3;
end

public_vars.kf.R = gnss_cov;
public_vars.kf.Q = zeros(2);

public_vars.kf.gnss_mean = gnss_mean;
public_vars.kf.gnss_cov = gnss_cov;

public_vars.mu = gnss_mean;
public_vars.sigma = gnss_cov;

end
