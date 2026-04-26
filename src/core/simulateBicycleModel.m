function [theory, kpis] = simulateBicycleModel(data, cfg)
%SIMULATEBICYCLEMODEL Calculate theoretical vehicle response using a linear bicycle model.
%
% Outputs:
%   theory: Table with theoretical yaw_rate and lat_accel
%   kpis:   Table with theoretical gains and understeer gradient

v = data.vehicle_speed_mps;
delta_sw = data.steering_wheel_angle_deg;
delta_w = deg2rad(delta_sw / cfg.veh.ratio); % Wheel angle in radians

% Vehicle parameters
m  = cfg.veh.m;
L  = cfg.veh.L;
lf = cfg.veh.lf;
lr = cfg.veh.lr;
Cf = cfg.veh.Cf;
Cr = cfg.veh.Cr;

% 1. Calculate Understeer Gradient Kus [rad/(m/s^2)]
% Kus > 0: Understeer
% Kus < 0: Oversteer
Kus = (m/L) * (lr/Cf - lf/Cr);

% 2. Steady-State Gains
% Yaw Rate Gain: r / delta_w = V / (L + Kus * V^2)
% We use the mean speed for a general gain, or point-by-point for the signal
yawRateGain_theory = v ./ (L + Kus * v.^2);

% 3. Theoretical signals (Steady-State approximation)
yaw_rate_theory_radps = yawRateGain_theory .* delta_w;
yaw_rate_theory_degps = rad2deg(yaw_rate_theory_radps);

lat_accel_theory_mps2 = v .* yaw_rate_theory_radps;

theory = table(yaw_rate_theory_degps, lat_accel_theory_mps2, ...
    'VariableNames', {'yaw_rate_theory_degps', 'lat_accel_theory_mps2'});

% 4. Summary KPIs
Kus_deg_g = rad2deg(Kus) * 9.81; % Understeer gradient in deg/g (standard unit)
kpis = table(Kus, Kus_deg_g, 'VariableNames', {'Kus_rad_mps2', 'Kus_deg_g'});

end
