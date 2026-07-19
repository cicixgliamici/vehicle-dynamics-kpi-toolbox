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

% Phase/Delay Analysis (using cross-correlation for robustness)
steeringToYawDelay_s = xcorrDelay(time, steer, yaw);
steeringToLatAccelDelay_s = xcorrDelay(time, steer, latacc);

kpis = table(steeringAmplitude_deg, peakSteeringRate_degps, ...
    steeringToYawDelay_s, steeringToLatAccelDelay_s);

end

function delay = xcorrDelay(time, u, y)
    % Estimate transport delay using normalized cross-correlation.
    %
    % This is the standard DSP method: it finds the time lag that
    % maximizes the linear cross-correlation between input u and output y.
    % Implemented without toolbox dependencies.
    %
    % Positive delay means y lags behind u (causal system, expected).
    dt = mean(diff(time));
    n  = length(u);

    % Normalize to zero-mean, unit variance for a scale-independent result
    u_norm = (u - mean(u)) / (std(u) + eps);
    y_norm = (y - mean(y)) / (std(y) + eps);

    % Cap search at 1 second of lag (physiologically max for a vehicle)
    maxLag = min(round(1.0 / dt), n - 1);
    lags   = -maxLag:maxLag;
    R      = zeros(1, numel(lags));

    for k = 1:numel(lags)
        lag = lags(k);
        if lag >= 0
            R(k) = sum(u_norm(1:n-lag) .* y_norm(lag+1:n));
        else
            R(k) = sum(u_norm(1-lag:n)  .* y_norm(1:n+lag));
        end
    end

    [~, peakIdx] = max(R);
    delayLag = lags(peakIdx);
    delay    = delayLag * dt;

    % Sanity check: discard physiologically impossible delays
    if delay < 0 || delay > 1.0
        delay = NaN;
    end
end
