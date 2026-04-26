function out = resampleVehicleData(data, targetSampleTime_s)
%RESAMPLEVEHICLEDATA Resample all numeric signals to a fixed sample time.

newTime = (data.time_s(1):targetSampleTime_s:data.time_s(end))';
out = table(newTime, 'VariableNames', {'time_s'});

for i = 1:width(data)
    col = data.Properties.VariableNames{i};
    if strcmp(col, "time_s")
        continue;
    end
    out.(col) = interp1(data.time_s, data.(col), newTime, "linear", "extrap");
end
end
