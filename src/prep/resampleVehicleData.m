function out = resampleVehicleData(data, targetSampleTime_s)
%RESAMPLEVEHICLEDATA Resample all numeric channels to a uniform time grid.
%
%   out = resampleVehicleData(data, targetSampleTime_s) takes a table with
%   a potentially non-uniform 'time_s' column and interpolates all numeric
%   signals onto a strictly uniform grid with spacing targetSampleTime_s.
%
%   Why resampling is necessary:
%       Real vehicle loggers (e.g. CANalyzer, INCA) can produce non-uniform
%       time steps due to CAN bus arbitration delays or asynchronous sensor
%       trigger rates. The Welch PSD estimator and any DFT-based analysis
%       requires equally spaced samples — non-uniform spacing would cause
%       spectral smearing and biased estimates.
%
%   Method:
%       1. Build a new time vector with uniform step Δt = targetSampleTime_s:
%              t* = [t_start : Δt : t_end]'
%       2. For each numeric column x(t), compute x(t*) via linear
%          interpolation with extrapolation at boundaries:
%              x(t*_k) = interp1(t, x, t*_k, 'linear', 'extrap')
%
%   Why linear interpolation (not spline or sinc)?
%       Linear interpolation avoids ringing artefacts (Gibbs phenomenon)
%       that sinc/spline resampling can introduce near sharp transitions
%       (e.g. step-steer inputs). For signals pre-filtered at 5 Hz and
%       resampled to 100 Hz, linear error is negligible.
%
%   Inputs:
%       data               - Table with at least a 'time_s' column.
%       targetSampleTime_s - Desired sample period [s]. Default pipeline
%                            value: 0.01 s (100 Hz, from cfg.targetSampleTime_s).
%
%   Output:
%       out - Table with uniform time_s and all resampled numeric channels.
%
%   See also: REMOVEMISSINGSAMPLES, LOWPASSFILTERSIGNAL, MATH_REFERENCE §1.2

% 1. Define the new uniform time vector from the first to the last timestamp.
%    Using colon operator guarantees machine-precision uniform spacing.
newTime = (data.time_s(1) : targetSampleTime_s : data.time_s(end))';
out = table(newTime, 'VariableNames', {'time_s'});

% 2. Iterate through each column and interpolate numeric channels.
for i = 1:width(data)
    col = data.Properties.VariableNames{i};

    % The time axis is already handled — skip it.
    if strcmp(col, 'time_s')
        continue;
    end

    % Only interpolate numeric data; leave string or categorical columns alone.
    if isnumeric(data.(col))
        % 'extrap' prevents NaNs at the boundary samples caused by floating-
        % point rounding when newTime slightly exceeds the original time range.
        out.(col) = interp1(data.time_s, data.(col), newTime, 'linear', 'extrap');
    end
end
end
