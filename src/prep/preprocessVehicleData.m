function data = preprocessVehicleData(rawData, cfg)
%PREPROCESSVEHICLEDATA Run the full preprocessing pipeline on raw telemetry.
%
%   This function orchestrates the cleaning, resampling, and filtering
%   of the incoming data to ensure it is ready for KPI extraction.

% 1. Clean data: Replace NaNs or missing samples using interpolation
% This prevents crashes in filtering and numerical integration
data = removeMissingSamples(rawData);

% 2. Resample: Convert irregular time-steps to a fixed frequency (e.g., 100Hz)
% Essential for consistent frequency-domain analysis and filtering
data = resampleVehicleData(data, cfg.targetSampleTime_s);

% 3. Filtering: Remove high-frequency noise from critical sensors
% We use the list defined in the config to know which signals to filter
signalsToFilter = [ ...
    "steering_wheel_angle_deg", ...
    "yaw_rate_degps", ...
    "lateral_accel_mps2", ...
    "longitudinal_accel_mps2", ...
    "roll_rate_degps", ...
    "vertical_accel_mps2" ...
];

for i = 1:numel(signalsToFilter)
    col = signalsToFilter(i);
    % Check if the column exists to avoid errors on partial datasets
    if ismember(col, data.Properties.VariableNames)
        % Apply the moving average filter based on the configured window
        data.(col) = lowPassFilterSignal(data.(col), cfg.lowPass.windowSamples);
    end
end

end
