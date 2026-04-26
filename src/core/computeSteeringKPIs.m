function kpis = computeSteeringKPIs(data, events, cfg)
%COMPUTESTEERINGKPIS Compute steering-response and delay KPIs.
%
%   KPIS = COMPUTESTEERINGKPIS(DATA, EVENTS, CFG) extracts steering dynamics
%   from the vehicle DATA table. Focuses on reaction speeds and timing.
%
%   Inputs:
%       data   - Table with time_s, steering_wheel_angle_deg, 
%                yaw_rate_degps, lateral_accel_mps2.
%       events - Table with start_time_s and end_time_s of maneuvers.
%       cfg    - Configuration struct.
%
%   Outputs:
%       kpis   - Table with: (steeringAmplitude, peakSteeringRate, 
%                steeringToYawDelay, steeringToLatAccelDelay).

% Validation
if isempty(data) || isempty(events)
    error('vdt:computeSteeringKPIs:InvalidInput', 'Data or Events table is empty.');
end

window = data.time_s >= events.start_time_s(1) & data.time_s <= events.end_time_s(1);
if ~any(window), window = true(height(data), 1); end

time = data.time_s(window);
steer = data.steering_wheel_angle_deg(window);
yaw = data.yaw_rate_degps(window);
latacc = data.lateral_accel_mps2(window);

% Dynamics
steeringAmplitude_deg = max(abs(steer));
dt = mean(diff(time));
if isnan(dt) || dt <= 0, dt = 0.01; end % Fallback

steeringRate_degps = gradient(steer, dt);
peakSteeringRate_degps = max(abs(steeringRate_degps));

% Phase/Delay Analysis
steeringToYawDelay_s = estimateDelay(time, steer, yaw);
steeringToLatAccelDelay_s = estimateDelay(time, steer, latacc);

kpis = table(steeringAmplitude_deg, peakSteeringRate_degps, ...
    steeringToYawDelay_s, steeringToLatAccelDelay_s);

end

function delay = estimateDelay(time, inputSignal, outputSignal)
    % Estimate delay using peak absolute derivative times.
    dt = mean(diff(time));
    inputDerivative = abs(gradient(inputSignal, dt));
    outputAbs = abs(outputSignal);

    [~, inputIdx] = max(inputDerivative);
    [~, outputIdx] = max(outputAbs);

    delay = time(outputIdx) - time(inputIdx);
    if delay < 0 || delay > 1.0 % Unrealistic delay for a vehicle
        delay = NaN;
    end
end
