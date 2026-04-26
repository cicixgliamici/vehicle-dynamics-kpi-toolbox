function data = removeMissingSamples(data)
%REMOVEMISSINGSAMPLES Fill missing values using linear interpolation.

for i = 1:width(data)
    col = data.Properties.VariableNames{i};
    if isnumeric(data.(col))
        data.(col) = fillmissing(data.(col), "linear", "EndValues", "nearest");
    end
end
end
