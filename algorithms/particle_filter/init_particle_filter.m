function [public_vars] = init_particle_filter(read_only_vars, public_vars)
    %INIT_PARTICLE_FILTER Summary of this function goes here

    N = read_only_vars.max_particles;
    xmin = read_only_vars.map.limits(1);
    xmax = read_only_vars.map.limits(3);
    ymin = read_only_vars.map.limits(2);
    ymax = read_only_vars.map.limits(4);

    public_vars.particles = zeros(N, 3);
    public_vars.particles(:, 1) = xmin + (xmax - xmin) * rand(N, 1);
    public_vars.particles(:, 2) = ymin + (ymax - ymin) * rand(N, 1);
    public_vars.particles(:, 3) = -pi + 2*pi * rand(N, 1);
end
