function plotTimeSeries(data, outputFile)
%PLOTTIMESERIES Plot the key vehicle dynamics signals as time series.
%
%   plotTimeSeries(data) displays a 3-panel figure showing:
%       Panel 1 — Steering Wheel Angle [deg]:   the driver input signal.
%       Panel 2 — Yaw Rate [deg/s] and Lateral Acceleration (×10) [m/s²]:
%                 overlaid on the same axes to show their correlation.
%                 Lateral acceleration is scaled ×10 for visibility since
%                 the two signals have different physical units and ranges.
%       Panel 3 — Vertical Acceleration [m/s²]: ride-comfort diagnostic.
%
%   Inputs:
%       data       - Preprocessed vehicle data table.
%       outputFile - (Optional) Full path to save the figure as PNG.
%                    If omitted, the figure is shown interactively.
%
%   See also: PLOTYAWRESPONSE, PLOTFREQUENCYRESPONSE, PLOTRESULTS

% Use 'Visible','off' to suppress the GUI window during batch/CI runs.
% The figure is rendered in memory and saved directly to disk.
figure('Visible', 'off');
t = data.time_s;

% Panel 1: Driver input — steering wheel angle
subplot(3,1,1);
plot(t, data.steering_wheel_angle_deg, 'LineWidth', 1.5);
ylabel('Steer [deg]');
grid on;
title('Vehicle Dynamics Time Series');

% Panel 2: Primary lateral dynamics signals
% Lateral acceleration is scaled ×10 to align its visual range with yaw rate
% (yaw rate is typically 0–40 deg/s; lat. acc. is 0–4 m/s² → ×10 = 0–40).
subplot(3,1,2);
plot(t, data.yaw_rate_degps, 'LineWidth', 1.5); hold on;
plot(t, data.lateral_accel_mps2 * 10, '--');
ylabel('Yaw [deg/s] / LatAcc×10 [m/s²]');
legend('Yaw Rate', 'Lat Accel (×10)', 'Location', 'best');
grid on;

% Panel 3: Vertical acceleration for ride comfort assessment
subplot(3,1,3);
plot(t, data.vertical_accel_mps2, 'LineWidth', 1.5);
ylabel('Vert Accel [m/s²]');
xlabel('Time [s]');
grid on;

if nargin > 1 && ~isempty(outputFile)
    saveas(gcf, outputFile);
end
close(gcf);

end
