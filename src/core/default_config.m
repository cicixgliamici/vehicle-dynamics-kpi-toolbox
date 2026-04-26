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

% --- Vehicle Physical Parameters (Bicycle Model) ---
cfg.veh.m  = 1600;       % Mass [kg]
cfg.veh.Iz = 2500;       % Yaw moment of inertia [kg*m^2]
cfg.veh.lf = 1.2;        % Distance CG to front axle [m]
cfg.veh.lr = 1.4;        % Distance CG to rear axle [m]
cfg.veh.L  = cfg.veh.lf + cfg.veh.lr; % Total wheelbase [m]
cfg.veh.Cf = 80000;      % Cornering stiffness front [N/rad] (per axle)
cfg.veh.Cr = 100000;     % Cornering stiffness rear [N/rad] (per axle)
cfg.veh.ratio = 15;      % Steering ratio (Steering Wheel Angle / Wheel Angle)

end
