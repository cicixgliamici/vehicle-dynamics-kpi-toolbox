function plotTimeSeries(data, outputFile)
%PLOTTIMESERIES Plot main vehicle dynamics signals.

figure('Visible', 'off'); % Hide if running in batch
t = data.time_s;

subplot(3,1,1);
plot(t, data.steering_wheel_angle_deg, 'LineWidth', 1.5);
ylabel('Steer [deg]');
grid on;
title('Vehicle Dynamics Time Series');

subplot(3,1,2);
plot(t, data.yaw_rate_degps, 'LineWidth', 1.5);
hold on;
plot(t, data.lateral_accel_mps2 * 10, '--'); % Scaled for visibility
ylabel('Yaw [deg/s] / LatAcc*10');
legend('Yaw Rate', 'Lat Accel (x10)');
grid on;

subplot(3,1,3);
plot(t, data.vertical_accel_mps2, 'LineWidth', 1.5);
ylabel('Vert Accel [m/s^2]');
xlabel('Time [s]');
grid on;

if nargin > 1 && ~isempty(outputFile)
    saveas(gcf, outputFile);
end

end
