function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

% N = size(particle_measurements, 1);
% weights = ones(N,1) / N;
    N = size(particle_measurements, 1);
    if N == 0
        weights = [];
        return;
    end
    
    M = min(size(particle_measurements, 2), numel(lidar_distances));
    if M == 0
        weights = ones(N,1) / N;
        return;
    end
    
    expected = lidar_distances(1:M);
    predicted = particle_measurements(:, 1:M);
    measurement_noise_sigma = 0.25;
    min_weight = 1e-10;
    inv_sigma2 = 1 / (measurement_noise_sigma^2);
    valid_expected = isfinite(expected) & expected > 0;
    
    weights = zeros(N, 1);
    for i = 1:N
        valid = valid_expected & isfinite(predicted(i,:)) & predicted(i,:) > 0;
        if any(valid)
            error_vec = predicted(i, valid) - expected(valid);
            weights(i) = max(exp(-0.5 * inv_sigma2 * sum(error_vec.^2)), min_weight);
        else
            weights(i) = min_weight;
        end
    end
    
    weight_sum = sum(weights);
    if ~isfinite(weight_sum) || weight_sum <= 0
        weights = ones(N,1) / N;
    else
        weights = weights / weight_sum;
    end
end

