function validateVehicleData(data)
%VALIDATEVEHICLEDATA Validate required columns and basic signal properties.

cfg = default_config();
columns = string(data.Properties.VariableNames);

missing = setdiff(cfg.requiredColumns, columns);
if ~isempty(missing)
    error("validateVehicleData:MissingColumns", ...
        "Missing required columns: %s", strjoin(missing, ", "));
end

if any(diff(data.time_s) <= 0)
    error("validateVehicleData:InvalidTime", ...
        "time_s must be strictly increasing.");
end

for i = 1:numel(cfg.requiredColumns)
    col = cfg.requiredColumns(i);
    if ~isnumeric(data.(col))
        error("validateVehicleData:NonNumericColumn", ...
            "Column %s must be numeric.", col);
    end
end
end
