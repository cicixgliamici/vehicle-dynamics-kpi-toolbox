function data = removeMissingSamples(data)
%REMOVEMISSINGSAMPLES Fill missing values (NaN) using linear interpolation.
%
%   Data dropouts are common in vehicle telemetry due to CAN bus load 
%   or wireless transmission issues. This function ensures a continuous signal.

for i = 1:width(data)
    col = data.Properties.VariableNames{i};
    % We only process numeric columns
    if isnumeric(data.(col))
        % Fillmissing with 'linear' handles gaps in the middle.
        % 'EndValues','nearest' prevents NaNs at the very start or end of the file.
        data.(col) = fillmissing(data.(col), "linear", "EndValues", "nearest");
    end
end

end
