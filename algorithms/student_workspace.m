function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

numberOfMeassurements = 150;
global lidar_measurements
global gnss_measurements
global i

if isempty(i)
    i = 1;
    lidar_measurements = zeros(numberOfMeassurements, 8);
    gnss_measurements  = zeros(numberOfMeassurements, 2);
end

if i <= 150
    lidar_measurements(i, :) = read_only_vars.lidar_distances;
    gnss_measurements(i, :) = read_only_vars.gnss_position;
    i = i+1;
end
%% Task 2
if i == 150
    lidarStandartDev = std(lidar_measurements, 0, "all")
    lidarChannelsStandartDev(1) = std(lidar_measurements(:,1), 0, "all");
    lidarChannelsStandartDev(2) = std(lidar_measurements(:,2), 0, "all");
    lidarChannelsStandartDev(3) = std(lidar_measurements(:,3), 0, "all");
    lidarChannelsStandartDev(4) = std(lidar_measurements(:,4), 0, "all");
    lidarChannelsStandartDev(5) = std(lidar_measurements(:,5), 0, "all");
    lidarChannelsStandartDev(6) = std(lidar_measurements(:,6), 0, "all");
    lidarChannelsStandartDev(7) = std(lidar_measurements(:,7), 0, "all");
    lidarChannelsStandartDev(8) = std(lidar_measurements(:,8), 0, "all")
    
    gnssStandartDev = std(gnss_measurements, 0, "all")
    gnssChannelsStandartDev(1) = std(gnss_measurements(:, 1), 0, "all")
    gnssChannelsStandartDev(2) = std(gnss_measurements(:, 2), 0, "all")
    
    figure('Name','LiDAR histograms');
    for lidarChannel = 1:8
        subplot(2, 4, lidarChannel);
        histogram(lidar_measurements(:, lidarChannel), 20);
        xlabel('Meassurements'); ylabel('Count');
    end

    figure('Name','Gnss histograms');
    for gnssChannel = 1:2
        subplot(1, 2, gnssChannel );
        histogram(gnss_measurements(:, gnssChannel ), 20);
    end

end

%% Task 3
if i == 150
    lidar_covariance = cov(lidar_measurements)
    gnss_covariance = cov(gnss_measurements)

    mainDiagLidar = diag(lidar_covariance)
    mainDiagGnaa = diag(gnss_covariance)
    
    lidarChannelsStandartDevSquared = lidarChannelsStandartDev.^2
    gnssChannelsStandartDevSquared = gnssChannelsStandartDev.^2


    i = i+1;
end

%% Task 4



%%
% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end

