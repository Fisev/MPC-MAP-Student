function [weights] = weight_particles(particle_measurements, lidar_distances)
    %WEIGHT_PARTICLES Summary of this function goes here

    numberOfParticals = size(particle_measurements, 1);
        
    measurementNoiseSigma = 0.25;
    minWeight = 1e-8;
    validoParticals = isfinite(lidar_distances) & lidar_distances > 0;
    
    weights = zeros(numberOfParticals, 1);
    for i = 1:numberOfParticals
        valid = validoParticals & isfinite(particle_measurements(i,:)) & particle_measurements(i,:) > 0;
        if any(valid)
            difference = particle_measurements(i, valid) - lidar_distances(valid);
            weights(i) = max(exp(-0.5 / (measurementNoiseSigma^2) * sum(difference.^2)), minWeight);
        else
            weights(i) = minWeight;
        end
    end
    
    weight_sum = sum(weights);

    weights = weights / weight_sum;
end

