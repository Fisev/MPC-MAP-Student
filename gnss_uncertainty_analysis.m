% GNSS_UNCERTAINTY_ANALYSIS
%
% Standalone helper for Task 1 of Assignment 4.
%
% Loads the outdoor_1 map, samples the GNSS sensor many times at the
% nominal start pose [2, 2, pi/2] and reports the estimated noise
% statistics (mean, covariance, standard deviation, one-sigma ellipse).
% A scatter plot of the samples together with the true position, the
% sample mean and a 1-sigma / 2-sigma confidence ellipse is drawn for
% the report.
%
% Run from the repository root (same folder as main.m).

close all;
clear;
clc;

% -----------------------------------------------------------------------
% Environment
% -----------------------------------------------------------------------
environment_setup;

map       = load_map('maps/outdoor_1.txt');
true_pose = [2, 2, pi/2];
N         = 1000;

% -----------------------------------------------------------------------
% Collect samples
% -----------------------------------------------------------------------
samples = nan(N, 2);
for i = 1:N
    g = gnss_measure(true_pose, map.gnss_denied);
    samples(i, :) = g(1:2);
end
valid   = all(isfinite(samples), 2);
samples = samples(valid, :);

% -----------------------------------------------------------------------
% Statistics
% -----------------------------------------------------------------------
mu_gnss    = mean(samples, 1);
cov_gnss   = cov(samples);
std_gnss   = sqrt(diag(cov_gnss)).';
rho        = cov_gnss(1,2) / (std_gnss(1) * std_gnss(2));
bias       = mu_gnss - true_pose(1:2);

fprintf('--- GNSS uncertainty @ (%.2f, %.2f) ---\n', true_pose(1), true_pose(2));
fprintf('Number of valid samples: %d / %d\n', size(samples,1), N);
fprintf('Mean               : [% .4f, % .4f]\n', mu_gnss(1), mu_gnss(2));
fprintf('Bias (mean - true) : [% .4f, % .4f]\n', bias(1), bias(2));
fprintf('Std. dev (sigma_x, sigma_y) : [%.4f, %.4f]\n', std_gnss(1), std_gnss(2));
fprintf('Correlation rho_xy : % .4f\n', rho);
fprintf('Covariance matrix  :\n');
disp(cov_gnss);

% -----------------------------------------------------------------------
% Plot
% -----------------------------------------------------------------------
theta  = linspace(0, 2*pi, 200);
circle = [cos(theta); sin(theta)];
[V, D] = eig(cov_gnss);
axes1  = V * sqrt(D) * circle;        % 1-sigma ellipse
axes2  = 2 * axes1;                   % 2-sigma ellipse

figure('Name', 'GNSS uncertainty', 'Color', 'w');
hold on; grid on; axis equal;
scatter(samples(:,1), samples(:,2), 8, [0.2 0.4 0.8], 'filled', ...
        'MarkerFaceAlpha', 0.4);
plot(true_pose(1), true_pose(2), 'k+', 'MarkerSize', 14, 'LineWidth', 2);
plot(mu_gnss(1), mu_gnss(2), 'rx', 'MarkerSize', 14, 'LineWidth', 2);
plot(mu_gnss(1) + axes1(1,:), mu_gnss(2) + axes1(2,:), 'r-',  'LineWidth', 2);
plot(mu_gnss(1) + axes2(1,:), mu_gnss(2) + axes2(2,:), 'r--', 'LineWidth', 1.2);

xlabel('x [m]'); ylabel('y [m]');
title(sprintf('GNSS samples at true position [%.2f, %.2f]  (N = %d)', ...
              true_pose(1), true_pose(2), size(samples,1)));
legend({'GNSS samples', 'True position', 'Sample mean', ...
        '1\sigma ellipse', '2\sigma ellipse'}, 'Location', 'best');
