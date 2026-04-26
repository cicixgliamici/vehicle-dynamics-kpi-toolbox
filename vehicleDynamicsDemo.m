function kpis = computeSteeringKPIs(data, events, cfg)
%COMPUTESTEERINGKPIS Compute steering-response KPIs.

window = data.time_s >= events.start_time_s(1) & data.time_s <= events.end_time_s(1);
if ~any(window)
    window = true(height(data), 1);
end

time = data.time_s(window);
steer = data.steering_wheel_angle_deg(window);
yaw = data.yaw_rate_degps(window);
latacc = data.lateral_accel_mps2(window);

steeringAmplitude_deg = max(abs(steer));
steeringRate_degps = gradient(steer, mean(diff(time)));
peakSteeringRate_degps = max(abs(steeringRate_degps));

steeringToYawDelay_s = estimateDelay(time, steer, yaw);
steeringToLatAccelDelay_s = estimateDelay(time, steer, latacc);

kpis = table(steeringAmplitude_deg, peakSteeringRate_degps, ...
    steeringToYawDelay_s, steeringToLatAccelDelay_s);
end

function delay = estimateDelay(time, inputSignal, outputSignal)
%ESTIMATEDELAY Estimate delay using peak absolute derivative times.
inputDerivative = abs(gradient(inputSignal, mean(diff(time))));
outputAbs = abs(outputSignal);

[~, inputIdx] = max(inputDerivative);
[~, outputIdx] = max(outputAbs);

delay = time(outputIdx) - time(inputIdx);
if delay < 0
    delay = NaN;
end
end
