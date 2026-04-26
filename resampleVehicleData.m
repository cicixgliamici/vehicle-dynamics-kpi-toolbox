function generateSyntheticData()
%GENERATESYNTHETICDATA Generate synthetic vehicle dynamics datasets.
%
% The signals are simplified but shaped to resemble common test/simulation
% data used in objective vehicle dynamics analysis.

outDir = fullfile("data", "synthetic");
if ~exist(outDir, "dir")
    mkdir(outDir);
end

fs = 100;                  % Hz
Ts = 1 / fs;
time_s = (0:Ts:20)';
n = numel(time_s);
vehicle_speed_mps = 20 * ones(n, 1); % 72 km/h

% 1. Step steer
steer = zeros(n, 1);
steer(time_s >= 3) = 30;
yaw = firstOrderResponse(time_s, steer, 0.55, 0.35);
latacc = 0.22 * yaw + 0.05 * randn(n, 1);
writeDataset(fullfile(outDir, "step_steer.csv"), time_s, vehicle_speed_mps, steer, yaw, latacc);

% 2. Sine steer
steer = zeros(n, 1);
idx = time_s >= 3 & time_s <= 13;
steer(idx) = 20 * sin(2*pi*0.4*(time_s(idx)-3));
yaw = firstOrderResponse(time_s, steer, 0.45, 0.25);
latacc = 0.20 * yaw + 0.05 * randn(n, 1);
writeDataset(fullfile(outDir, "sine_steer.csv"), time_s, vehicle_speed_mps, steer, yaw, latacc);

% 3. Double lane change simplified
steer = 25 * exp(-0.5*((time_s-5)/0.45).^2) ...
      - 28 * exp(-0.5*((time_s-7)/0.50).^2) ...
      + 12 * exp(-0.5*((time_s-9)/0.60).^2);
yaw = firstOrderResponse(time_s, steer, 0.50, 0.30);
latacc = 0.24 * yaw + 0.08 * randn(n, 1);
writeDataset(fullfile(outDir, "double_lane_change.csv"), time_s, vehicle_speed_mps, steer, yaw, latacc);

% 4. Ride bump: mainly vertical acceleration event
steer = zeros(n, 1);
yaw = 0.05 * randn(n, 1);
latacc = 0.03 * randn(n, 1);
vertical = 0.08 * randn(n, 1) + 2.5 * exp(-0.5*((time_s-8)/0.15).^2) ...
         - 1.4 * exp(-0.5*((time_s-8.4)/0.25).^2);
writeDataset(fullfile(outDir, "ride_bump.csv"), time_s, vehicle_speed_mps, steer, yaw, latacc, vertical);

fprintf("Synthetic datasets generated in %s\n", outDir);
end

function y = firstOrderResponse(t, u, gain, tau)
%FIRSTORDERRESPONSE Simple discrete first-order response y' = (gain*u-y)/tau.
y = zeros(size(u));
for k = 2:numel(t)
    dt = t(k) - t(k-1);
    y(k) = y(k-1) + dt * ((gain * u(k) - y(k-1)) / tau);
end
y = y + 0.15 * randn(size(y));
end

function writeDataset(filename, time_s, vehicle_speed_mps, steering, yaw, latacc, vertical)
%WRITEDATASET Write a standard vehicle dynamics table to CSV.
if nargin < 7
    vertical = 0.08 * randn(size(time_s));
end

longitudinal_accel_mps2 = 0.02 * randn(size(time_s));
roll_rate_degps = 0.12 * latacc + 0.02 * randn(size(time_s));

T = table(time_s, vehicle_speed_mps, steering, yaw, latacc, ...
    longitudinal_accel_mps2, roll_rate_degps, vertical, ...
    'VariableNames', { ...
        'time_s', ...
        'vehicle_speed_mps', ...
        'steering_wheel_angle_deg', ...
        'yaw_rate_degps', ...
        'lateral_accel_mps2', ...
        'longitudinal_accel_mps2', ...
        'roll_rate_degps', ...
        'vertical_accel_mps2'});

writetable(T, filename);
end
