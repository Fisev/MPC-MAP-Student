function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
    %COMPUTE_MEASUREMENTS Summary of this function goes here

    measurement = zeros(1, length(lidar_config));

    currentPose = pose(1:2);
    th = pose(3);

    numberOfBeams = length(lidar_config);

    for i = 1:numberOfBeams
        direction = mod(th + lidar_config(i) + pi, 2*pi) - pi;              %wrapToPi(th + lidar_config(i));     
        intersections = ray_cast(currentPose, map.walls, direction);
        if ~isempty(intersections)
            n = size(intersections, 1);
            dists = zeros(1, n);
            for j = 1:n
                dists(j) = sqrt(sum((intersections(j, :) - currentPose).^2));
            end
            measurement(i) = min(dists);
        end
    end
end
