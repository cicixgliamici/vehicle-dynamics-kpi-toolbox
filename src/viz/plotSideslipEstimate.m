function plotSideslipEstimate(data, kfResults, outputFile)
%PLOTSIDESLIPESTIMATE Plot the Kalman Filter sideslip angle estimation results.
%
%   plotSideslipEstimate(data, kfResults, outputFile) generates a 3-panel
%   diagnostic figure:
%
%   Panel 1 — Sideslip Angle β [deg]:
%     Estimated β with ±2σ confidence band (from P_trace).
%     β > 0: vehicle slipping to the right.
%
%   Panel 2 — Yaw Rate: Measured vs. KF-Estimated [deg/s]:
%     Validates filter tracking performance.
%
%   Panel 3 — KF Innovation (Measurement Residual) [deg/s]:
%     A white-noise innovation confirms the filter is consistent.
%     Systematic trends indicate model mismatch.
%
%   Inputs:
%       data       - Original preprocessed data table (needs time_s,
%                    yaw_rate_degps).
%       kfResults  - Table returned by estimateSideslip().
%       outputFile - Full path to save the figure as PNG.
%
%   See also: ESTIMATESIDESLIP

if nargin < 3 || isempty(outputFile)
    outputFile = '';
end

t           = kfResults.time_s;
beta        = kfResults.beta_est_deg;
r_est       = kfResults.r_est_degps;
P_tr        = kfResults.P_trace;
innov       = kfResults.innovation_degps;
r_meas      = data.yaw_rate_degps;

% 2-sigma confidence bound: sqrt(P_trace) gives combined std,
% we approximate σ_β ≈ sqrt(P_trace/2) for the marginal bound.
sigma_beta = rad2deg(sqrt(max(P_tr/2, 0)));
beta_upper =  beta + 2*sigma_beta;
beta_lower =  beta - 2*sigma_beta;

% --- Figure setup ---
fig = figure('Name', 'Kalman Filter — Sideslip Estimation', ...
             'Position', [100 100 900 700], 'Visible', 'off');

% ---- Panel 1: Sideslip angle with confidence band ----
ax1 = subplot(3,1,1);
fill([t; flipud(t)], [beta_upper; flipud(beta_lower)], ...
    [0.4 0.6 1.0], 'FaceAlpha', 0.25, 'EdgeColor', 'none', ...
    'DisplayName', '\pm2\sigma confidence'); hold on;
plot(t, beta, 'b', 'LineWidth', 1.8, 'DisplayName', '\beta_{est} (KF)');
plot(t, zeros(size(t)), 'k--', 'LineWidth', 0.8, 'HandleVisibility', 'off');
ylabel('\beta [deg]', 'FontWeight', 'bold');
title('Sideslip Angle Estimate — Linear Kalman Filter', 'FontSize', 11);
legend('Location', 'northeast');
grid on; box on;

% ---- Panel 2: Yaw rate measured vs estimated ----
ax2 = subplot(3,1,2);
plot(t, r_meas, 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2, ...
    'DisplayName', 'r_{meas} (IMU)'); hold on;
plot(t, r_est, 'r', 'LineWidth', 1.5, 'LineStyle', '--', ...
    'DisplayName', 'r_{est} (KF)');
ylabel('Yaw Rate [deg/s]', 'FontWeight', 'bold');
legend('Location', 'northeast');
grid on; box on;

% ---- Panel 3: Innovation (residual) ----
ax3 = subplot(3,1,3);
area(t, innov, 'FaceColor', [0.9 0.6 0.2], 'FaceAlpha', 0.5, ...
    'EdgeColor', [0.8 0.4 0]);
plot(t, zeros(size(t)), 'k-', 'LineWidth', 0.8);
ylabel('Innovation [deg/s]', 'FontWeight', 'bold');
xlabel('Time [s]', 'FontWeight', 'bold');
title('KF Innovation (y - C\cdot\hat{x}^{-}) — should be zero-mean', ...
    'FontSize', 9);
grid on; box on;

% Link x-axes for synchronised zooming
linkaxes([ax1, ax2, ax3], 'x');

% --- Save ---
if ~isempty(outputFile)
    exportgraphics(fig, outputFile, 'Resolution', 150);
end
close(fig);

end
