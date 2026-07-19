function data = removeMissingSamples(data)
%REMOVEMISSINGSAMPLES Fill missing values (NaN) using linear interpolation.
%
%   data = removeMissingSamples(data) scans every numeric column of the
%   input table and replaces NaN values with linearly interpolated estimates
%   from neighbouring valid samples.
%
%   Why data dropouts occur:
%       Vehicle telemetry systems transmit over CAN bus, which operates at
%       fixed message rates (e.g. 10–100 Hz per signal). Under high bus load,
%       messages can be dropped. Wireless data loggers add further packet loss.
%       The result is isolated NaN "holes" in otherwise continuous signals.
%
%   Strategy:
%       - Interior gaps: piecewise linear interpolation between the two
%         nearest valid neighbours on either side of the gap.
%             x(t_k) = x(t_a) + (x(t_b)-x(t_a))/(t_b-t_a) * (t_k-t_a)
%       - Boundary gaps (start or end of file): nearest-neighbour
%         extrapolation — hold the first/last valid value constant.
%         This avoids introducing a spurious linear trend at the edges.
%
%   Limitations:
%       - Large gaps (> 0.5 s) should be flagged rather than silently filled;
%         linear interpolation of a long gap introduces artificial dynamics.
%       - Non-numeric columns (e.g. event labels) are left untouched.
%
%   Input / Output:
%       data - MATLAB table with arbitrary columns. Numeric columns with
%              NaN values are filled in-place; all others are unchanged.
%
%   See also: RESAMPLEVEHICLEDATA, VALIDATEVEHICLEDATA, MATH_REFERENCE §1.1

for i = 1:width(data)
    col = data.Properties.VariableNames{i};

    % Only process numeric columns; skip strings, categoricals, etc.
    if isnumeric(data.(col))
        % 'linear': piecewise linear interpolation for interior gaps.
        % 'EndValues','nearest': nearest-neighbour hold for boundary gaps
        %   — prevents a linear extrapolation slope at the file edges.
        data.(col) = fillmissing(data.(col), 'linear', 'EndValues', 'nearest');
    end
end

end
