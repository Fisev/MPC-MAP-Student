function [new_particles] = resample_particles(particles, weights, map_limits, random_count)
    %RESAMPLE_PARTICLES Summary of this function goes here
    if nargin < 4
        random_count = 50;
    end

    N = size(particles, 1);
    new_particles = zeros(size(particles));

    weights = weights(:) / sum(weights);

    % resampling
    weightsSet = [0; cumsum(weights)];
    u = rand / N;
    currentWeightSet = 1;
    for i = 1:N
        ui = u + (i - 1) / N;
        while ui > weightsSet(currentWeightSet + 1)
            currentWeightSet = currentWeightSet + 1;
        end
        new_particles(i, :) = particles(currentWeightSet, :);
    end

    %add random particles.

    random_count = min(random_count, N);

    if random_count > 0
        random_idx   = randperm(N, random_count);
        xmin = map_limits(1); xmax = map_limits(3);
        ymin = map_limits(2); ymax = map_limits(4);
        new_particles(random_idx, 1) = xmin + (xmax - xmin) * rand(random_count, 1);
        new_particles(random_idx, 2) = ymin + (ymax - ymin) * rand(random_count, 1);
        new_particles(random_idx, 3) = -pi + 2*pi * rand(random_count, 1);
    end
end
