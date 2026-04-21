function [new_particles] = resample_particles(particles, weights)
    %RESAMPLE_PARTICLES Summary of this function goes here

    numberOfParticals = length(particles);
    new_particles = zeros(size(particles));
    
    weightsSet = [0; cumsum(weights)];
    u = rand/numberOfParticals;
    currentWeightSet = 1;
    for i = 1:numberOfParticals
        ui = u + (i-1)/numberOfParticals;
        while ui > weightsSet(currentWeightSet+1)
            currentWeightSet = currentWeightSet + 1;
        end
        new_particles(i, :) = particles(currentWeightSet, :);
    end

    numberOfRandomParticals = 100;
    random_idx = randperm(numberOfParticals, numberOfRandomParticals);
    
    new_particles(random_idx, 1) = 20 * rand(numberOfRandomParticals, 1);
    new_particles(random_idx, 2) = 15 * rand(numberOfRandomParticals, 1);
    new_particles(random_idx, 3) = 2*pi * rand(numberOfRandomParticals, 1);
end

