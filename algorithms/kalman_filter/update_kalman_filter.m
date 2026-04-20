function [mu, sigma, kf] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here

mu = public_vars.mu;
sigma = public_vars.sigma;
kf = public_vars.kf;

if ~public_vars.kf_enabled
    return;
end

if ~kf.use_known_initial_pose && ~kf.is_initialized
    if ~any(isnan(read_only_vars.gnss_position))
        kf.gnss_samples = [kf.gnss_samples; read_only_vars.gnss_position];
    end
    
    n = size(kf.gnss_samples, 1);
    if n >= kf.init_samples
        gnss_mu = mean(kf.gnss_samples, 1);
        gnss_cov = cov(kf.gnss_samples);
        if any(isnan(gnss_cov), 'all') || rank(gnss_cov) < 2
            gnss_cov = diag(max(var(kf.gnss_samples, 0, 1), 1e-4));
        end
        
        kf.Q = gnss_cov;
        mu = [gnss_mu(1); gnss_mu(2); 0];
        sigma = diag([1, 1, pi^2]);
        sigma(1:2, 1:2) = gnss_cov;
        kf.is_initialized = 1;
    else
        if n > 0
            gnss_mu = mean(kf.gnss_samples, 1);
            mu = [gnss_mu(1); gnss_mu(2); 0];
        end
        sigma = diag([10, 10, pi^2]);
        return;
    end
end

% I. Prediction
u = public_vars.motion_vector(:);
[mu, sigma] = ekf_predict(mu, sigma, u, kf, read_only_vars.sampling_period);

% II. Measurement
z = read_only_vars.gnss_position(:);
[mu, sigma] = kf_measure(mu, sigma, z, kf);

end
