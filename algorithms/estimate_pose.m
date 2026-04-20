function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

estimated_pose = nan(1,3);

if isfield(public_vars, 'kf_enabled') && public_vars.kf_enabled && isfield(public_vars, 'mu') && numel(public_vars.mu) == 3
    estimated_pose = public_vars.mu(:)';
    return;
end

if isfield(public_vars, 'pf_enabled') && public_vars.pf_enabled && ~isempty(public_vars.particles)
    particles = public_vars.particles(:, 1:3);
    theta = atan2(mean(sin(particles(:,3))), mean(cos(particles(:,3))));
    estimated_pose = [mean(particles(:,1)), mean(particles(:,2)), theta];
end

end
