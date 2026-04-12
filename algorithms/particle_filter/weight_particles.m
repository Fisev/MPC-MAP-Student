function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

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

weights = zeros(N, 1);
for i = 1:N
    valid = isfinite(expected) & isfinite(predicted(i,:)) & expected > 0 & predicted(i,:) > 0;
    if any(valid)
        error_vec = predicted(i, valid) - expected(valid);
        weights(i) = exp(-0.5 * sum((error_vec ./ measurement_noise_sigma).^2));
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
