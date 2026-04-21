function [public_vars] = init_particle_filter(read_only_vars, public_vars)
    %INIT_PARTICLE_FILTER Summary of this function goes here

    public_vars.particles = [];
    
    public_vars.particles = rand([read_only_vars.max_particles, 3]);
    public_vars.particles(:, 3) = public_vars.particles(:, 3).* 2*pi;
    public_vars.particles(:, 1) = public_vars.particles(:, 1)*read_only_vars.discrete_map.limits(3);
    public_vars.particles(:, 2) = public_vars.particles(:, 2)*read_only_vars.discrete_map.limits(4);

end

