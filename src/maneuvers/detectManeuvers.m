function events = detectManeuvers(data, cfg)
%DETECTMANEUVERS Automatically identify active maneuvers in the dataset.
%
%   Uses a threshold-based trigger on the absolute steering wheel angle
%   to find when the driver is performing a test.

% --- Parameters ---
steer = abs(data.steering_wheel_angle_deg);
threshold = cfg.maneuver.steeringThreshold_deg;
minDuration_s = cfg.maneuver.minDuration_s;

% --- Trigger Logic ---
% 1. Find samples where steering is above the 'active' threshold
active = steer > threshold;

% 2. Find transitions (Start = 0 to 1, End = 1 to 0)
% We pad with 0 to catch maneuvers that might start at the very first sample
starts = find(diff([0; active]) == 1);
ends = find(diff([active; 0]) == -1);

% --- Filtering by Duration ---
% We calculate the time elapsed between each start and end
% This ignores accidental "twitches" or noise spikes that are too short
durations = data.time_s(ends) - data.time_s(starts);
valid = durations >= minDuration_s;

starts = starts(valid);
ends = ends(valid);

% --- Output Generation ---
if isempty(starts)
    % Fallback: If no steering activity is found, we assume the whole file 
    % is the maneuver (useful for constant radius tests or simple simulations)
    events = table(data.time_s(1), data.time_s(end), "all", ...
        'VariableNames', {'start_time_s', 'end_time_s', 'type'});
else
    % Create a table with the identified windows
    events = table(data.time_s(starts), data.time_s(ends), ...
        repmat("steering_event", numel(starts), 1), ...
        'VariableNames', {'start_time_s', 'end_time_s', 'type'});
end

end
