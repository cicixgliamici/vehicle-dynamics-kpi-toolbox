function [frResults, frMetrics] = computeFrequencyResponse(data, cfg)
%COMPUTEFREQUENCYRESPONSE Estimate vehicle transfer functions and coherence.
%
%   [frResults, frMetrics] = computeFrequencyResponse(data, cfg) calculates
%   the gain, phase, and coherence between steering input and vehicle 
%   responses (Yaw Rate and Lateral Acceleration).
%
%   Inputs:
%       data: table with time_s, steering_wheel_angle_deg, yaw_rate_degps, etc.
%       cfg:  struct with freq.windowLength, freq.overlap, etc.
%
%   Outputs:
%       frResults: table with frequency axis and complex TFs/coherence.
%       frMetrics: table with scalar KPIs (bandwidth, peak gain, etc.).

% 1. Extract signals and remove DC offset (detrend)
dt = mean(diff(data.time_s));
fs = 1/dt;

u = detrend(data.steering_wheel_angle_deg);
y1 = detrend(data.yaw_rate_degps);
y2 = detrend(data.lateral_accel_mps2);

% 2. Setup Welch-like estimation
L = cfg.freq.windowLength;
overlap = cfg.freq.overlap;
nfft = L;
window = hann(L);

% Frequency axis
f = (0:nfft/2) * (fs/nfft);
idx = f >= cfg.freq.minFreq_Hz & f <= cfg.freq.maxFreq_Hz;
f = f(idx)';

% Initialization of PSD/CSD accumulators
Puu = zeros(size(f));
Puy1 = zeros(size(f), 'like', 1i);
Puy2 = zeros(size(f), 'like', 1i);
Py1y1 = zeros(size(f));
Py2y2 = zeros(size(f));

% Segmented averaging
step = L - overlap;
numSegments = floor((length(u) - L) / step) + 1;

for i = 1:numSegments
    startIdx = (i-1)*step + 1;
    endIdx = startIdx + L - 1;
    
    u_seg = u(startIdx:endIdx) .* window;
    y1_seg = y1(startIdx:endIdx) .* window;
    y2_seg = y2(startIdx:endIdx) .* window;
    
    U = fft(u_seg, nfft);
    Y1 = fft(y1_seg, nfft);
    Y2 = fft(y2_seg, nfft);
    
    % Keep only positive frequencies
    U = U(1:nfft/2+1);
    Y1 = Y1(1:nfft/2+1);
    Y2 = Y2(1:nfft/2+1);
    
    % Accumulate (taking index filter into account)
    Puu = Puu + abs(U(idx)).^2;
    Py1y1 = Py1y1 + abs(Y1(idx)).^2;
    Py2y2 = Py2y2 + abs(Y2(idx)).^2;
    Puy1 = Puy1 + conj(U(idx)) .* Y1(idx);
    Puy2 = Puy2 + conj(U(idx)) .* Y2(idx);
end

% Average
Puu = Puu / numSegments;
Py1y1 = Py1y1 / numSegments;
Py2y2 = Py2y2 / numSegments;
Puy1 = Puy1 / numSegments;
Puy2 = Puy2 / numSegments;

% 3. Calculate Transfer Functions and Coherence
H_yaw = Puy1 ./ Puu;
H_lat = Puy2 ./ Puu;

Coh_yaw = abs(Puy1).^2 ./ (Puu .* Py1y1);
Coh_lat = abs(Puy2).^2 ./ (Puu .* Py2y2);

% 4. Build results table
frResults = table(f, H_yaw, H_lat, Coh_yaw, Coh_lat, ...
    'VariableNames', {'Frequency_Hz', 'TF_Yaw', 'TF_LatAcc', 'Coh_Yaw', 'Coh_Lat'});

% 5. Extract Scalar Metrics
frMetrics = extractFrMetrics(f, H_yaw, cfg, 'Yaw');
frMetricsLat = extractFrMetrics(f, H_lat, cfg, 'LatAcc');
frMetrics = [frMetrics frMetricsLat];

end

function metrics = extractFrMetrics(f, H, cfg, suffix)
    gain = abs(H);
    phase = unwrap(angle(H)) * 180/pi;
    
    % DC Gain (approx at min frequency)
    dcGain = gain(1);
    
    % Bandwidth (-3dB from DC gain)
    targetGain = dcGain * 10^(cfg.freq.bandwidthThreshold_dB/20);
    bwIdx = find(gain < targetGain, 1);
    if isempty(bwIdx)
        bandwidth = f(end);
    else
        bandwidth = f(bwIdx);
    end
    
    % Resonant Peak
    [peakGain, peakIdx] = max(gain);
    peakFreq = f(peakIdx);
    
    % Phase lag at 1Hz (Standard KPI)
    [~, f1Idx] = min(abs(f - 1.0));
    phaseLag_1Hz = phase(f1Idx);
    
    metrics = table(bandwidth, peakGain, peakFreq, phaseLag_1Hz, ...
        'VariableNames', {['Bandwidth_Hz_' suffix], ['PeakGain_' suffix], ...
        ['PeakFreq_Hz_' suffix], ['PhaseLag_1Hz_deg_' suffix]});
end

function w = hann(L)
    w = 0.5 * (1 - cos(2*pi*(0:L-1)'/(L-1)));
end

function y = detrend(x)
    n = length(x);
    if n < 2, y = x; return; end
    t = (0:n-1)';
    p = polyfit(t, x, 1);
    y = x - polyval(p, t);
end
