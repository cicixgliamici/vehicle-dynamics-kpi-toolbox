function filteredSignal = lowPassFilterSignal(signal, windowSamples)
%LOWPASSFILTERSIGNAL Apply a zero-phase moving-average low-pass filter.
%
%   filteredSignal = lowPassFilterSignal(signal, windowSamples) smooths the
%   input column vector using a centred symmetric moving-average window of
%   length M (always forced to odd so the window is symmetric around n=0).
%
%   Algorithm:
%       y[n] = (1/M) * sum_{k=-(M-1)/2}^{(M-1)/2} x[n+k]
%
%   Frequency response (Dirichlet kernel):
%       |H(f)| = (1/M) * |sin(pi*f*M/fs) / sin(pi*f/fs)|
%
%   Approximate -3 dB cutoff: f_c ≈ 0.443 * fs / M
%   For M=9, fs=100 Hz → f_c ≈ 4.9 Hz (attenuates noise above ~5 Hz).
%
%   Zero-phase property: the symmetric window introduces NO group delay,
%   which is essential for preserving the relative timing between signals
%   used in cross-correlation delay estimation.
%
%   NaN handling: 'omitnan' averages available neighbours around NaN gaps,
%   removing the need for a separate gap-filling pass on borderline dropouts.
%
%   Inputs:
%       signal        - Column vector of signal samples (Nx1 double).
%       windowSamples - Moving-average window length (positive integer).
%                       Even values are rounded up to the next odd number.
%
%   Output:
%       filteredSignal - Filtered column vector, same size as input.
%
%   See also: REMOVEMISSINGSAMPLES, RESAMPLEVEHICLEDATA, MATH_REFERENCE §1.3

arguments
    signal        (:,1) double
    windowSamples (1,1) double {mustBeInteger, mustBePositive}
end

% Force odd window length for a symmetric (zero-phase) filter kernel.
% An even-length window would be asymmetric and introduce a half-sample delay.
if mod(windowSamples, 2) == 0
    windowSamples = windowSamples + 1;
end

% movmean with 'omitnan' computes the mean over available (non-NaN) neighbours,
% effectively acting as both a smoother and a local gap-filler.
filteredSignal = movmean(signal, windowSamples, 'omitnan');
end
