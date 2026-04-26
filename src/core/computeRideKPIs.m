function kpis = computeRideKPIs(data, events, cfg)
%COMPUTERIDEKPIS Compute ride-comfort and vertical dynamics KPIs.
%
%   KPIS = COMPUTERIDEKPIS(DATA, EVENTS, CFG) analyzes vertical acceleration
%   and body movements to quantify ride quality.
%
%   Inputs:
%       data   - Table with time_s, vertical_accel_mps2.
%       events - Table with maneuver windows.
%       cfg    - Configuration struct.
%
%   Outputs:
%       kpis   - Table with: (peakToPeakVertAcc, rmsVertAcc).

% Validation
if isempty(data), error('vdt:computeRideKPIs:InvalidInput', 'Data table is empty.'); end

window = data.time_s >= events.start_time_s(1) & data.time_s <= events.end_time_s(1);
if ~any(window), window = true(height(data), 1); end

vertAcc = data.vertical_accel_mps2(window);

% Peak-to-peak and RMS are standard for basic ride comfort
% We use the manual RMS formula to avoid dependency on Signal Processing Toolbox
peakToPeakVertAcc_mps2 = max(vertAcc) - min(vertAcc);
rmsVertAcc_mps2 = sqrt(mean((vertAcc - mean(vertAcc)).^2));

kpis = table(peakToPeakVertAcc_mps2, rmsVertAcc_mps2);

end
