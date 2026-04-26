function plotYawResponse(data, kpis, outputFile)
%PLOTYAWRESPONSE Plot yaw rate vs steering angle for handling analysis.

figure('Visible', 'off');
hold on;
plot(data.steering_wheel_angle_deg, data.yaw_rate_degps, '.', 'Color', [0.7 0.7 0.7], 'DisplayName', 'Measured');

if ismember('yaw_rate_theory_degps', data.Properties.VariableNames)
    % Plot theoretical curve sorted by steering angle for a clean line
    [steerSorted, idx] = sort(data.steering_wheel_angle_deg);
    plot(steerSorted, data.yaw_rate_theory_degps(idx), 'r-', 'LineWidth', 2, 'DisplayName', 'Bicycle Model');
end

xlabel('Steering Wheel Angle [deg]');
ylabel('Yaw Rate [deg/s]');
title('Handling: Yaw Response Characteristic');
grid on;
legend('Location', 'best');

text(0.1, 0.9, sprintf('Exp Gain: %.3f deg/s/deg', kpis.yawRateGain_1ps(1)), 'Units', 'normalized');

if nargin > 2 && ~isempty(outputFile)
    saveas(gcf, outputFile);
end

end
