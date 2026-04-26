function kpis = computeHandlingKPIs(data, events, cfg)
%COMPUTEHANDLINGKPIS Compute basic handling/yaw-response KPIs.

window = data.time_s >= events.start_time_s(1) & data.time_s <= events.end_time_s(1);
if ~any(window)
    window = true(height(data), 1);
end

steer = data.steering_wheel_angle_deg(window);
yaw = data.yaw_rate_degps(window);
latacc = data.lateral_accel_mps2(window);
time = data.time_s(window);

peakLatAccel_mps2 = max(abs(latacc));
peakYawRate_degps = max(abs(yaw));
peakSteer_deg = max(abs(steer));

yawRateGain_1ps = safeDivide(peakYawRate_degps, peakSteer_deg);
latAccelGain_mps2_per_deg = safeDivide(peakLatAccel_mps2, peakSteer_deg);

finalYaw = mean(yaw(max(1, end-round(cfg.kpi.steadyStateWindow_s/mean(diff(time)))):end));
targetYaw = cfg.kpi.responsePercentage * finalYaw;
responseTime_s = estimateResponseTime(time, yaw, targetYaw, events.start_time_s(1));
settlingTime_s = estimateSettlingTime(time, yaw, finalYaw, cfg.kpi.settlingBandPercentage, events.start_time_s(1));

kpis = table(peakLatAccel_mps2, peakYawRate_degps, yawRateGain_1ps, ...
    latAccelGain_mps2_per_deg, responseTime_s, settlingTime_s);
end

function v = safeDivide(a, b)
if abs(b) < eps
    v = NaN;
else
    v = a / b;
end
end

function rt = estimateResponseTime(time, signal, target, t0)
idx = find(abs(signal) >= abs(target), 1, "first");
if isempty(idx)
    rt = NaN;
else
    rt = time(idx) - t0;
end
end

function st = estimateSettlingTime(time, signal, finalValue, bandPct, t0)
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
