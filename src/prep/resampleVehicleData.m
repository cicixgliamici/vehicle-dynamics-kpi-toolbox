function out = resampleVehicleData(data, targetSampleTime_s)
%RESAMPLEVEHICLEDATA Resample all numeric signals to a fixed sample time.

% 1. Define the new uniform time vector from start to end
newTime = (data.time_s(1):targetSampleTime_s:data.time_s(end))';
out = table(newTime, 'VariableNames', {'time_s'});

% 2. Iterate through each column to interpolate
for i = 1:width(data)
    col = data.Properties.VariableNames{i};
    % Skip the time column as we already defined the new one
    if strcmp(col, "time_s")
        continue;
    end
    % Only resample numeric data (ignore strings or categorical)
    if isnumeric(data.(col))
        % Use linear interpolation with extrapolation to handle edge samples
        out.(col) = interp1(data.time_s, data.(col), newTime, "linear", "extrap");
    end
end
end
