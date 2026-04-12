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
end

