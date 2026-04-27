function filteredSignal = lowPassFilterSignal(signal, windowSamples)
%LOWPASSFILTERSIGNAL Apply a moving-average low-pass filter to a signal.
%
%   filteredSignal = lowPassFilterSignal(signal, windowSamples)
%   smooths the input vector with a centered moving-average window.
%
%   This implementation intentionally avoids requiring extra toolboxes.

arguments
    signal (:,1) double
    windowSamples (1,1) double {mustBeInteger, mustBePositive}
end

% Ensure odd window length for symmetric filtering.
if mod(windowSamples, 2) == 0
    windowSamples = windowSamples + 1;
end

filteredSignal = movmean(signal, windowSamples, 'omitnan');
end
