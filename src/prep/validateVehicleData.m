function validateVehicleData(data)
%VALIDATEVEHICLEDATA Validate required columns and basic signal properties.
%
%   This is the 'Gatekeeper' function. It ensures the data structure is
%   compatible with the physics engine of the toolbox.

cfg = default_config();
columns = string(data.Properties.VariableNames);

% 1. Column Integrity: Check for missing physical signals
% We use 'setdiff' to find which required signals are not in the table
missing = setdiff(cfg.requiredColumns, columns);
if ~isempty(missing)
    error("validateVehicleData:MissingColumns", ...
        "Missing required columns: %s", strjoin(missing, ", "));
end

% 2. Time Integrity: Check for monotonicity
% Resampling and interpolation will fail if time is not strictly increasing
if any(diff(data.time_s) <= 0)
    error("validateVehicleData:InvalidTime", ...
        "time_s must be strictly increasing. Check for data gaps or duplicate timestamps.");
end

% 3. Data Type Integrity
% Ensure signals are numeric to allow mathematical operations
for i = 1:numel(cfg.requiredColumns)
    col = cfg.requiredColumns(i);
    if ~isnumeric(data.(col))
        error("validateVehicleData:NonNumericColumn", ...
            "Column %s must be numeric.", col);
    end
end
end
