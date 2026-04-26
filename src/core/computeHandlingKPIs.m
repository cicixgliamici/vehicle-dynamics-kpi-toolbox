function kpis = computeHandlingKPIs(data, events, cfg)
%COMPUTEHANDLINGKPIS Compute basic handling and yaw-response KPIs.
%
%   KPIS = COMPUTEHANDLINGKPIS(DATA, EVENTS, CFG) extracts handling metrics
%   from the vehicle DATA table based on the first event in the EVENTS table.
%
%   Inputs:
%       data   - Table containing time_s, steering_wheel_angle_deg, 
%                yaw_rate_degps, lateral_accel_mps2.
%       events - Table with start_time_s and end_time_s of maneuvers.
%       cfg    - Configuration struct (see default_config.m).
%
%   Outputs:
%       kpis   - 1xN Table containing extracted handling metrics:
%                (peakLatAccel, peakYawRate, yawRateGain, latAccelGain,
%                 responseTime, settlingTime).
%
%   Example:
%       kpis = computeHandlingKPIs(data, events, cfg);
%
%   See also: COMPUTESTEERINGKPIS, COMPUTERIDEKPIS

% --- Input Validation ---
if isempty(data) || height(data) < 2
    error('vdt:computeHandlingKPIs:InvalidData', 'Data table is empty or too short.');
end
if isempty(events)
    error('vdt:computeHandlingKPIs:NoEvents', 'Events table is empty. Run maneuver detection first.');
end

% --- Window Selection ---
% We focus on the first detected event for these specific handling KPIs
tStart = events.start_time_s(1);
tEnd = events.end_time_s(1);

window = data.time_s >= tStart & data.time_s <= tEnd;
if ~any(window)
    warning('vdt:computeHandlingKPIs:EmptyWindow', 'No data found for the specified event time range. Using full dataset.');
    window = true(height(data), 1);
end

% --- Signal Extraction ---
steer = data.steering_wheel_angle_deg(window);
yaw = data.yaw_rate_degps(window);
latacc = data.lateral_accel_mps2(window);
time = data.time_s(window);

% --- Metric Computation ---
peakLatAccel_mps2 = max(abs(latacc));
peakYawRate_degps = max(abs(yaw));
peakSteer_deg = max(abs(steer));

% Gains (using Peak-to-Peak/Steady-State simplified approach)
yawRateGain_1ps = safeDivide(peakYawRate_degps, peakSteer_deg);
latAccelGain_mps2_per_deg = safeDivide(peakLatAccel_mps2, peakSteer_deg);

% Time Response Metrics
finalYaw = mean(yaw(max(1, end-round(cfg.kpi.steadyStateWindow_s/mean(diff(time)))):end));
targetYaw = cfg.kpi.responsePercentage * finalYaw;

responseTime_s = estimateResponseTime(time, yaw, targetYaw, tStart);
settlingTime_s = estimateSettlingTime(time, yaw, finalYaw, cfg.kpi.settlingBandPercentage, tStart);

% --- Result Construction ---
kpis = table(peakLatAccel_mps2, peakYawRate_degps, yawRateGain_1ps, ...
    latAccelGain_mps2_per_deg, responseTime_s, settlingTime_s);

end

% --- Helper Functions (Private) ---

function v = safeDivide(a, b)
    % Avoid division by zero
    if abs(b) < eps
        v = NaN;
    else
        v = a / b;
    end
end

function rt = estimateResponseTime(time, signal, target, t0)
    % Estimate time to reach X% of final response
    idx = find(abs(signal) >= abs(target), 1, "first");
    if isempty(idx)
        rt = NaN;
    else
        rt = time(idx) - t0;
    end
end

function st = estimateSettlingTime(time, signal, finalValue, bandPct, t0)
    % Estimate time to stay within X% of final value
    if abs(finalValue) < eps
        st = NaN;
        return;
    end
    band = abs(finalValue) * bandPct;
    inside = abs(signal - finalValue) <= band;
    st = NaN;
    for k = 1:numel(signal)
        if all(inside(k:end))
            st = time(k) - t0;
            return;
        end
    end
end
