% validateRobustness.m
% Validate the toolbox accuracy against noisy data.

clear; clc; close all;
run(fullfile('..', 'startup.m'));

cfg = default_config();

% 1. Create a "Ground Truth" scenario (Perfect data)
fs = 100;
t = (0:1/fs:10)';
steer = zeros(size(t));
steer(t > 2 & t < 8) = 30 * sin(2*pi*0.2*(t(t > 2 & t < 8)-2));

% Expected gains (Simple linear relation)
trueGain = 0.52;
yaw = trueGain * steer;
latacc = 0.22 * yaw;

% Save clean and noisy versions
dataClean = table(t, 20*ones(size(t)), steer, yaw, latacc, zeros(size(t)), zeros(size(t)), zeros(size(t)), ...
    'VariableNames', cfg.requiredColumns);

% Add heavy noise
dataNoisy = dataClean;
noiseLevel = 0.8; % Deg/s or M/s^2 noise
dataNoisy.yaw_rate_degps = dataNoisy.yaw_rate_degps + noiseLevel * randn(size(t));
dataNoisy.lateral_accel_mps2 = dataNoisy.lateral_accel_mps2 + (noiseLevel*0.5) * randn(size(t));

% 2. Process both
dataProcessed = preprocessVehicleData(dataNoisy, cfg);
events = detectManeuvers(dataProcessed, cfg);
kpisNoisy = computeAllKPIs(dataProcessed, events, cfg);
kpisClean = computeAllKPIs(dataClean, events, cfg);

% 3. Compare Results
fprintf('--- ROBUSTNESS VALIDATION ---\n');
fprintf('Ground Truth Gain: %.4f\n', trueGain);
fprintf('Clean KPI Gain:    %.4f\n', kpisClean.yawRateGain_1ps);
fprintf('Noisy KPI Gain:    %.4f (Error: %.2f%%)\n', ...
    kpisNoisy.yawRateGain_1ps, abs(kpisNoisy.yawRateGain_1ps - trueGain)/trueGain * 100);

% 4. Visualization
figure;
subplot(2,1,1);
hold on;
plot(t, dataNoisy.yaw_rate_degps, 'Color', [0.8 0.8 0.8], 'DisplayName', 'Noisy Raw');
plot(t, dataProcessed.yaw_rate_degps, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered');
plot(t, dataClean.yaw_rate_degps, 'r--', 'DisplayName', 'Ground Truth');
ylabel('Yaw Rate [deg/s]');
legend; grid on; title('Robustness: Filtering Performance');

subplot(2,1,2);
errorSignal = dataProcessed.yaw_rate_degps - dataClean.yaw_rate_degps;
plot(t, errorSignal, 'k');
ylabel('Residual Error'); xlabel('Time [s]');
grid on;
