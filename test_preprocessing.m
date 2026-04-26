function y = lowPassFilterSignal(x, windowSamples)
%LOWPASSFILTERSIGNAL Lightweight moving-average low-pass filter.
%
% This avoids requiring Signal Processing Toolbox and keeps the MVP portable
% across MATLAB Online, MATLAB Desktop, and many student licenses.

if windowSamples <= 1
    y = x;
    return;
end

windowSamples = max(1, round(windowSamples));
kernel = ones(windowSamples, 1) / windowSamples;
y = conv(x, kernel, "same");
end
