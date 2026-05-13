function [estimated_pose] = estimate_pose(public_vars, read_only_vars)
    %ESTIMATE_POSE Summary of this function goes here

    if isfield(public_vars, 'particle_pose') && ~isempty(public_vars.particle_pose) && all(isfinite(public_vars.particle_pose))
        estimated_pose = public_vars.particle_pose;
    elseif isfield(public_vars, 'mu') && ~isempty(public_vars.mu)
        estimated_pose = public_vars.mu';
    else
        estimated_pose = nan(1, 3);
    end


    limits = read_only_vars.map.limits; % [xmin ymin xmax ymax]
    
    % put X to limits
    estimated_pose(1) = max(limits(1) + 0.1, min(limits(3) - 0.1, estimated_pose(1)));
    
    %put y to limits
    estimated_pose(2) = max(limits(2) + 0.1, min(limits(4) - 0.1, estimated_pose(2)));
end
