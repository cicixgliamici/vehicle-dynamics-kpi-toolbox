function cfg = default_config()
%DEFAULT_CONFIG Return default configuration for the processing pipeline.
%
% The configuration is intentionally stored in a struct so that the same
% scripts can be reused with different sampling rates, filter cut-offs, and
% KPI thresholds.

cfg = struct();

% Target sample time after resampling.
cfg.targetSampleTime_s = 0.01; % 100 Hz

% Low-pass filter settings.
% This MVP uses a lightweight moving-average low-pass filter to avoid
% requiring extra MATLAB toolboxes.
cfg.lowPass.windowSamples = 9;

% Signals to be low-pass filtered during preprocessing.
% Centralised here so that preprocessVehicleData.m does not need
% to hardcode signal names (single source of truth).
cfg.lowPass.signalsToFilter = [ ...
    "steering_wheel_angle_deg", ...
    "yaw_rate_degps", ...
    "lateral_accel_mps2", ...
    "longitudinal_accel_mps2", ...
    "roll_rate_degps", ...
    "vertical_accel_mps2" ...
];

% Maneuver detection.
cfg.maneuver.steeringThreshold_deg = 1.0;
cfg.maneuver.minDuration_s = 0.5;

% KPI settings.
cfg.kpi.steadyStateWindow_s = 1.0;
cfg.kpi.responsePercentage = 0.9;
cfg.kpi.settlingBandPercentage = 0.05;

% Required columns in input tables.
cfg.requiredColumns = [ ...
    "time_s", ...
    "vehicle_speed_mps", ...
    "steering_wheel_angle_deg", ...
    "yaw_rate_degps", ...
    "lateral_accel_mps2", ...
    "longitudinal_accel_mps2", ...
    "roll_rate_degps", ...
    "vertical_accel_mps2" ...
];

% --- Vehicle Physical Parameters (Bicycle Model) ---
cfg.veh.m  = 1600;       % Mass [kg]
cfg.veh.Iz = 2500;       % Yaw moment of inertia [kg*m^2]
cfg.veh.lf = 1.2;        % Distance CG to front axle [m]
cfg.veh.lr = 1.4;        % Distance CG to rear axle [m]
cfg.veh.L  = cfg.veh.lf + cfg.veh.lr; % Total wheelbase [m]
cfg.veh.Cf = 80000;      % Cornering stiffness front [N/rad] (per axle)
cfg.veh.Cr = 100000;     % Cornering stiffness rear [N/rad] (per axle)
cfg.veh.ratio = 15;      % Steering ratio (Steering Wheel Angle / Wheel Angle)

% --- Frequency Analysis Parameters ---
cfg.freq.windowLength = 256;  % FFT window size (samples)
cfg.freq.overlap = 128;       % Window overlap (samples)
cfg.freq.minFreq_Hz = 0.1;    % Start frequency for analysis
cfg.freq.maxFreq_Hz = 5.0;    % End frequency for analysis
cfg.freq.bandwidthThreshold_dB = -3; % Threshold for bandwidth calculation

% --- Kalman Filter Parameters (Sideslip Estimation) ---
% Process noise Q: models uncertainty in the bicycle model itself.
%   Q(1,1) — variance on β dynamics (small: we trust the physics)
%   Q(2,2) — variance on r dynamics (slightly larger: yaw can be noisy)
cfg.kf.Q  = diag([1e-4, 1e-3]);

% Measurement noise R: models IMU yaw-rate sensor noise.
%   Typical MEMS gyro noise ≈ 0.5 deg/s → variance ≈ (0.5·π/180)² rad²/s²
cfg.kf.R  = (0.5 * pi/180)^2;

% Initial state covariance P0:
%   Large P0(1,1): we have NO prior knowledge of β at t=0.
%   Moderate P0(2,2): yaw rate is initialised from the first measurement.
cfg.kf.P0 = diag([0.01, 0.05]);

end
