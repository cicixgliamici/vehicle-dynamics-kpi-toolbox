function plotYawResponse(data, kpis, outputFile)
%PLOTYAWRESPONSE Plot the yaw rate vs steering angle handling characteristic.
%
%   plotYawResponse(data, kpis, outputFile) generates the classic
%   "handling diagram" — a scatter plot of yaw rate vs steering wheel angle.
%
%   Physical interpretation:
%       For a linear vehicle, yaw rate r is proportional to SWA δ:
%           r = G_r * δ    (G_r = yaw rate gain [s⁻¹])
%       The scatter cloud should cluster around a straight line through the
%       origin. Deviation from linearity indicates:
%           - Nonlinear tyre behaviour (saturation at high δ)
%           - Speed variation within the dataset
%           - Transient overshoot / undershoot
%
%   Overlay — Bicycle Model Prediction:
%       If the table contains the 'yaw_rate_theory_degps' column (appended by
%       simulateBicycleModel), the theoretical steady-state curve is plotted
%       in red. Comparing measured vs theoretical gain reveals:
%           - Understeer (measured gain < theory): real Kus > assumed Kus
%           - Oversteer  (measured gain > theory): real Kus < assumed Kus
%
%   Sorting the theoretical curve by steering angle (rather than by time)
%   is necessary to draw a clean line. Plotting vs time would produce a
%   tangled Lissajous figure for sine-sweep inputs.
%
%   Inputs:
%       data       - Data table with steering_wheel_angle_deg, yaw_rate_degps,
%                    and optionally yaw_rate_theory_degps.
%       kpis       - KPI table containing yawRateGain_1ps for annotation.
%       outputFile - (Optional) Full path to save the figure as PNG.
%
%   See also: SIMULATEBICYCLEMODEL, COMPUTEHANDLINGKPIS

figure('Visible', 'off');
hold on;

% Scatter of all measured samples — grey points to reduce visual clutter.
% Scatter is preferred over a line because time-ordering is not meaningful here.
plot(data.steering_wheel_angle_deg, data.yaw_rate_degps, '.', ...
    'Color', [0.7 0.7 0.7], 'DisplayName', 'Measured');

% Theoretical bicycle model curve (optional — only if column exists).
if ismember('yaw_rate_theory_degps', data.Properties.VariableNames)
    % Sort by steering angle to produce a monotone line (not a time-ordered
    % curve), avoiding the spaghetti appearance of dynamic manoeuvres.
    [steerSorted, idx] = sort(data.steering_wheel_angle_deg);
    plot(steerSorted, data.yaw_rate_theory_degps(idx), 'r-', ...
        'LineWidth', 2, 'DisplayName', 'Bicycle Model');
end

xlabel('Steering Wheel Angle [deg]');
ylabel('Yaw Rate [deg/s]');
title('Handling: Yaw Response Characteristic');
grid on;
legend('Location', 'best');

% Annotate with the experimental gain in the top-left corner.
% 'Units','normalized' keeps the text in a fixed position regardless of axis limits.
text(0.1, 0.9, sprintf('Exp. Gain: %.3f (deg/s)/deg', kpis.yawRateGain_1ps(1)), ...
    'Units', 'normalized', 'FontSize', 9);

if nargin > 2 && ~isempty(outputFile)
    saveas(gcf, outputFile);
end
close(gcf);

end
