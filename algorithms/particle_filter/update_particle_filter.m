function [public_vars] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here

    particles = public_vars.particles;

    % I. Prediction
    for i = 1:size(particles, 1)
        particles(i, :) = predict_pose(particles(i, :), public_vars.motion_vector, read_only_vars);
    end

    % II. Correction
    N = size(particles, 1);
    M = length(read_only_vars.lidar_config);
    measurements = zeros(N, M);
    for i = 1:N
        measurements(i, :) = compute_lidar_measurement(read_only_vars.map, particles(i, :), read_only_vars.lidar_config);
    end
    weights = weight_particles(measurements, read_only_vars.lidar_distances);

    [~, max_idx] = max(weights);
    best_p = particles(max_idx, :);

    dist = hypot(particles(:,1) - best_p(1), particles(:,2) - best_p(2));
    nearby = dist < 0.5;
    w_sub = weights(nearby) / sum(weights(nearby));
    p_sub = particles(nearby, :);

    public_vars.particle_pose = [ sum(p_sub(:,1) .* w_sub), sum(p_sub(:,2) .* w_sub), atan2(sum(sin(p_sub(:,3)) .* w_sub), sum(cos(p_sub(:,3)) .* w_sub))];

    public_vars.pf_neff = 1 / sum(weights.^2);

    if isfield(public_vars, 'pf_tight_mode') && public_vars.pf_tight_mode
        n_random = 0;
    elseif isfield(public_vars, 'pf_converged') && public_vars.pf_converged
        n_random = 60;
    else
        n_random = 100;
    end
    public_vars.particles = resample_particles(particles, weights, read_only_vars.map.limits, n_random);
    
end
