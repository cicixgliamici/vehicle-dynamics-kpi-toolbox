function [kfResults, kf_kpis] = estimateSideslip(data, cfg)
%ESTIMATESIDESLIP Estimate vehicle sideslip angle β using a Linear Kalman Filter.
%
%   The sideslip angle β is the angle between the vehicle's longitudinal axis
%   and its actual velocity vector. It quantifies lateral "sliding" of the
%   vehicle body and is a key indicator of handling limit proximity.
%   β is NOT directly measurable with standard IMUs and must be estimated.
%
%   This function implements a discrete-time Linear Kalman Filter (LKF) based
%   on the linearised 2-state bicycle model:
%
%     State:        x = [β; r]    (sideslip [rad], yaw rate [rad/s])
%     Input:        u = δ_f       (front wheel angle [rad])
%     Measurement:  y = r         (yaw rate measured by IMU [rad/s])
%
%   Continuous-time plant (evaluated at instantaneous speed v_k):
%
%     A = [-(Cf+Cr)/(m·v),       (Cr·lr - Cf·lf)/(m·v²) - 1 ]
%         [ (Cr·lr - Cf·lf)/Iz,  -(Cf·lf² + Cr·lr²)/(Iz·v)  ]
%
%     B = [ Cf/(m·v)  ]
%         [ Cf·lf/Iz  ]
%
%     C = [0, 1]   (yaw rate is directly observed)
%
%   Euler discretisation at each step (time-varying, handles speed changes):
%     Φ_k = I + A(v_k)·Δt
%     Γ_k = B(v_k)·Δt
%
%   Kalman recursion:
%     Predict:  x̂⁻ = Φ·x̂ + Γ·u,   P⁻ = Φ·P·Φᵀ + Q
%     Update:   K  = P⁻·Cᵀ·(C·P⁻·Cᵀ + R)⁻¹
%               x̂  = x̂⁻ + K·(y - C·x̂⁻)
%               P  = (I - K·C)·P⁻
%
%   Inputs:
%       data - Table with: time_s, vehicle_speed_mps,
%              steering_wheel_angle_deg, yaw_rate_degps.
%       cfg  - Configuration struct (see default_config.m).
%              Relevant fields: cfg.veh.*, cfg.kf.Q, cfg.kf.R, cfg.kf.P0.
%
%   Outputs:
%       kfResults - Table (N×5) with time series: time_s, beta_est_deg,
%                   r_est_degps, P_trace, innovation_degps.
%       kf_kpis   - Table (1×3) with scalar KPIs: maxSideslip_deg,
%                   rmsResidual_r_degps, meanSpeed_mps.
%
%   Example:
%       [kfResults, kf_kpis] = estimateSideslip(data, cfg);
%       disp(kf_kpis)
%
%   References:
%       Rajamani, R. (2011). Vehicle Dynamics and Control. Springer.
%       Welch & Bishop (2006). An Introduction to the Kalman Filter. UNC.
%
%   See also: SIMULATEBICYCLEMODEL, COMPUTEHANDLINGKPIS

% --- Input validation ---
if isempty(data) || height(data) < 4
    error('vdt:estimateSideslip:InvalidData', ...
        'Data table must have at least 4 rows.');
end
requiredCols = ["time_s", "vehicle_speed_mps", ...
                "steering_wheel_angle_deg", "yaw_rate_degps"];
for i = 1:numel(requiredCols)
    if ~ismember(requiredCols(i), data.Properties.VariableNames)
        error('vdt:estimateSideslip:MissingColumn', ...
            'Missing required column: %s', requiredCols(i));
    end
end

% --- Extract vehicle parameters ---
m  = cfg.veh.m;
Iz = cfg.veh.Iz;
lf = cfg.veh.lf;
lr = cfg.veh.lr;
Cf = cfg.veh.Cf;
Cr = cfg.veh.Cr;

% --- Extract signals ---
t      = data.time_s;
v      = data.vehicle_speed_mps;
% Guard against near-zero speed (model is singular at v→0)
v      = max(v, 1.0);

delta_f = deg2rad(data.steering_wheel_angle_deg / cfg.veh.ratio);
r_meas  = deg2rad(data.yaw_rate_degps);

dt = mean(diff(t));
n  = length(t);

% --- Kalman Filter tuning matrices ---
Q  = cfg.kf.Q;    % (2×2) Process noise covariance
R  = cfg.kf.R;    % (1×1) Measurement noise variance
I2 = eye(2);
C  = [0, 1];      % Observation: yaw rate is state x(2)

% --- Initialisation ---
% x0: β=0 (no lateral slip at start), r = first measurement
x = [0; r_meas(1)];
P = cfg.kf.P0;  % (2×2) Initial state error covariance

% --- Pre-allocate output arrays ---
beta_est_rad = zeros(n, 1);
r_est_rad    = zeros(n, 1);
P_trace      = zeros(n, 1);
innovation   = zeros(n, 1);

% =========================================================================
%   MAIN FILTER LOOP
%   Time-varying A, B matrices: recomputed at each step using v(k).
%   This makes the filter more accurate during acceleration/braking phases.
% =========================================================================
for k = 1:n
    v_k = v(k);
    u_k = delta_f(k);    % Front wheel angle input [rad]
    y_k = r_meas(k);     % Yaw rate measurement [rad/s]

    % --- Build time-varying continuous-time matrices at v_k ---
    A_c = [-(Cf+Cr)/(m*v_k),          (Cr*lr - Cf*lf)/(m*v_k^2) - 1; ...
           (Cr*lr - Cf*lf)/Iz,         -(Cf*lf^2 + Cr*lr^2)/(Iz*v_k)];

    B_c = [Cf/(m*v_k); ...
           Cf*lf/Iz];

    % --- Euler discretisation (stable at Δt = 0.01 s) ---
    Phi = I2 + A_c * dt;
    Gam = B_c * dt;

    % -----------------------------------------------------------------
    %   PREDICT STEP (a priori estimate)
    % -----------------------------------------------------------------
    x_pred = Phi * x + Gam * u_k;      % Propagate state
    P_pred = Phi * P * Phi' + Q;       % Propagate covariance (Riccati)

    % -----------------------------------------------------------------
    %   UPDATE STEP (a posteriori correction)
    % -----------------------------------------------------------------
    S   = C * P_pred * C' + R;         % Innovation covariance (scalar)
    K   = (P_pred * C') / S;           % Kalman gain  [2×1]
    innov = y_k - C * x_pred;          % Measurement residual (innovation)

    x = x_pred + K * innov;            % Corrected state estimate
    P = (I2 - K * C) * P_pred;         % Corrected covariance (Joseph form
                                        % preferred for numerical stability
                                        % but this is fine for 2×2)

    % Store results
    beta_est_rad(k) = x(1);
    r_est_rad(k)    = x(2);
    P_trace(k)      = trace(P);         % Total uncertainty: σ²_β + σ²_r
    innovation(k)   = innov;
end

% --- Convert to engineering units ---
beta_est_deg    = rad2deg(beta_est_rad);
r_est_degps     = rad2deg(r_est_rad);
innovation_degps = rad2deg(innovation);

% --- Build output table (time series) ---
kfResults = table(t, beta_est_deg, r_est_degps, P_trace, innovation_degps, ...
    'VariableNames', {'time_s', 'beta_est_deg', 'r_est_degps', ...
                      'P_trace', 'innovation_degps'});

% --- Scalar KPI table ---
maxSideslip_deg      = max(abs(beta_est_deg));
rmsResidual_r_degps  = sqrt(mean(innovation_degps.^2));
meanSpeed_mps        = mean(v);

kf_kpis = table(maxSideslip_deg, rmsResidual_r_degps, meanSpeed_mps, ...
    'VariableNames', {'maxSideslip_deg', 'rmsResidual_r_degps', 'meanSpeed_mps'});

end
