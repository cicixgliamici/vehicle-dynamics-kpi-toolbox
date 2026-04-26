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
end
