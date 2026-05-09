function generateLargeBatch(numDatasets)
%GENERATELARGEBATCH Generate a large number of varied synthetic datasets.
%   Used to demonstrate batch processing scalability and robustness.

if nargin < 1, numDatasets = 20; end

baseDir = ".";
[~, folderName] = fileparts(pwd);
if strcmp(folderName, 'scripts'), baseDir = ".."; end

outDir = fullfile(baseDir, "data", "synthetic_batch");
if ~exist(outDir, "dir"), mkdir(outDir); end

speeds = [10, 15, 20, 25, 30]; % m/s
noiseLevels = [0.01, 0.05, 0.1, 0.2]; % Noise standard deviation

fprintf('Generating %d datasets...\n', numDatasets);

for i = 1:numDatasets
    % Randomize parameters
    v = speeds(randi(length(speeds)));
    noise = noiseLevels(randi(length(noiseLevels)));
    maneuverType = randi(2); % 1: Step, 2: Sine
    
    fs = 100;
    t = (0:1/fs:15)';
    n = length(t);
    
    steer = zeros(n,1);
    if maneuverType == 1
        % Step
        steer(t > 2) = 20 + 10*rand(); 
        name = sprintf('batch_%03d_step_v%d_n%.2f.csv', i, v, noise);
    else
        % Sine
        f = 0.3 + 0.4*rand();
        steer(t > 2 & t < 12) = (15 + 10*rand()) * sin(2*pi*f*(t(t > 2 & t < 12)-2));
        name = sprintf('batch_%03d_sine_v%d_n%.2f.csv', i, v, noise);
    end
    
    % Simple first order response (approximate dynamics)
    tau = 0.2 + 0.2*rand();
    gain = 0.4 + 0.2*rand();
    
    yaw = zeros(size(steer));
    for k = 2:n
        yaw(k) = yaw(k-1) + (1/fs) * ((gain*steer(k) - yaw(k-1))/tau);
    end
    
    latacc = 0.2 * yaw; % Simple coupling
    
    % Add noise
    yaw = yaw + noise * randn(size(yaw));
    latacc = latacc + (noise*1.5) * randn(size(latacc));
    
    % Write to CSV
    T = table(t, v*ones(n,1), steer, yaw, latacc, ...
        randn(n,1)*0.01, randn(n,1)*0.05, randn(n,1)*0.08, ...
        'VariableNames', {'time_s', 'vehicle_speed_mps', 'steering_wheel_angle_deg', ...
        'yaw_rate_degps', 'lateral_accel_mps2', 'longitudinal_accel_mps2', ...
        'roll_rate_degps', 'vertical_accel_mps2'});
    
    writetable(T, fullfile(outDir, name));
end

fprintf('Successfully generated %d datasets in %s\n', numDatasets, outDir);

end
