# Architecture Guide — Vehicle Dynamics KPI Toolbox

## Overview

The toolbox implements a **linear, stateless data pipeline**: raw CSV telemetry flows through a sequence of transformations and ends as a structured table of scalar KPIs and optional plots.

```
CSV File
   │
   ▼
loadVehicleData()          [src/io]
   │  • File & encoding validation
   │  • Column name normalization (lowercase, underscores)
   │  • validateVehicleData() gate-check
   ▼
preprocessVehicleData()    [src/prep]
   │  • removeMissingSamples()   → interpolate NaN gaps
   │  • resampleVehicleData()    → uniform Δt (100 Hz default)
   │  • lowPassFilterSignal()    → moving-average anti-noise
   ▼
detectManeuvers()          [src/maneuvers]
   │  • Threshold trigger on |steering_wheel_angle_deg|
   │  • Duration filter (rejects accidental twitches)
   │  • Returns events table: [start_time_s, end_time_s, type]
   ▼
simulateBicycleModel()     [src/core]         ← steady-state ground truth
   │  • Computes: yaw_rate_theory, lat_accel_theory, Kus
   ▼
estimateSideslip()         [src/core]         ← Linear Kalman Filter
   │  • State: x = [β, r]ᵀ  (sideslip, yaw rate)
   │  • Time-varying A(v_k), B(v_k) matrices
   │  • Predict → Update cycle at each sample
   │  • Output: β_est(t), P_trace(t), innovation(t)
   ▼
computeAllKPIs()           [src/core]
   │  ├── computeHandlingKPIs()   → Yaw Gain, LatAcc Gain, t_response, t_settling
   │  ├── computeSteeringKPIs()  → Steer Amplitude, Rate, Steer→Yaw delay (xcorr)
   │  ├── computeRideKPIs()      → RMS & P2P vertical acceleration
   │  └── computeFrequencyResponse() → Welch PSD/CSD, Bode Gain/Phase, Coherence
   ▼
plotResults()              [src/viz]
   │  • plotTimeSeries()          → timeseries.png
   │  • plotYawResponse()         → yaw_response.png
   │  • plotFrequencyResponse()   → frequency_response.png
   ▼
exportKpiSummary()         [src/io]
      • CSV with timestamp, source_file, all KPI columns
      • Append-mode for batch runs
```

---

## Kalman Filter — Sideslip Angle Estimation

### Why Sideslip Matters
The *vehicle sideslip angle* β is the angle between the vehicle's heading and its actual velocity vector. It is a key indicator of how close the vehicle is to the handling limit. β is **not measurable** with standard IMU sensors and must be estimated.

### State-Space Formulation

**State:** $\mathbf{x} = [\beta,\ r]^T$ — sideslip [rad], yaw rate [rad/s]

**Continuous-time plant** (evaluated at instantaneous speed $v_k$):

$$A(v) = \begin{bmatrix} -\frac{C_f+C_r}{mv} & \frac{C_r l_r - C_f l_f}{mv^2} - 1 \\ \frac{C_r l_r - C_f l_f}{I_z} & -\frac{C_f l_f^2 + C_r l_r^2}{I_z v} \end{bmatrix}, \quad B(v) = \begin{bmatrix} \frac{C_f}{mv} \\ \frac{C_f l_f}{I_z} \end{bmatrix}$$

**Euler discretisation** at each step (stable at Δt = 0.01 s):

$$\Phi_k = I + A(v_k)\Delta t, \quad \Gamma_k = B(v_k)\Delta t$$

**Measurement model:** $C = [0, 1]$ (yaw rate directly observed by IMU)

### Kalman Recursion

| Step | Equation | Description |
|---|---|---|
| **Predict** | $\hat{\mathbf{x}}^-_k = \Phi_k \hat{\mathbf{x}}_{k-1} + \Gamma_k u_k$ | Propagate state |
| | $P^-_k = \Phi_k P_{k-1} \Phi_k^T + Q$ | Propagate covariance (Riccati) |
| **Update** | $K_k = P^-_k C^T (C P^-_k C^T + R)^{-1}$ | Compute Kalman gain |
| | $\hat{\mathbf{x}}_k = \hat{\mathbf{x}}^-_k + K_k(y_k - C\hat{\mathbf{x}}^-_k)$ | Correct state |
| | $P_k = (I - K_k C)\, P^-_k$ | Correct covariance |

### Observability Check

The observability matrix for $C = [0, 1]$:

$$\mathcal{O} = \begin{bmatrix} C \\ CA \end{bmatrix} = \begin{bmatrix} 0 & 1 \\ \frac{C_r l_r - C_f l_f}{I_z} & -\frac{C_f l_f^2 + C_r l_r^2}{I_z v} \end{bmatrix}$$

$\text{rank}(\mathcal{O}) = 2$ as long as $C_r l_r \neq C_f l_f$ (true for any real vehicle). The system is **fully observable from yaw rate alone**.

### Tuning Parameters (in `cfg.kf`)

| Parameter | Default | Meaning |
|---|---|---|
| `Q = diag([1e-4, 1e-3])` | — | Process noise: trust in the bicycle model |
| `R = (0.5·π/180)²` | ~7.6×10⁻⁵ rad²/s² | IMU noise: ~0.5 deg/s MEMS gyro std |
| `P0 = diag([0.01, 0.05])` | — | Initial uncertainty: β unknown, r semi-known |

> [!TIP]
> Increasing Q (larger model uncertainty) makes the filter more reactive to measurements. Decreasing R (more trust in the sensor) has the same effect. Start with the defaults and tune if the innovation residual shows systematic bias.

---

## Design Principles

| Principle | Implementation |
|---|---|
| **Zero toolbox dependency** | All DSP algorithms (Welch, Hann window, cross-corr) implemented from first principles using only base MATLAB |
| **Fail loudly, warn softly** | Hard failures use `error('vdt:...')` with structured IDs; recoverable issues use `warning()` |
| **Single source of truth** | All tunable parameters (filter window, thresholds, vehicle params) live in `default_config.m` |
| **Testable in isolation** | Each function accepts explicit inputs; no function reads globals or files internally (except `loadVehicleData`) |

---

## Vehicle Physical Parameters

The bicycle model uses the following default parameters, configurable in [`default_config.m`](../src/core/default_config.m):

| Parameter | Symbol | Value | Unit | Description |
|---|---|---|---|---|
| Vehicle mass | `m` | 1600 | kg | Total vehicle mass |
| Yaw inertia | `Iz` | 2500 | kg·m² | Moment of inertia about vertical axis |
| Front axle distance | `lf` | 1.2 | m | CG to front axle |
| Rear axle distance | `lr` | 1.4 | m | CG to rear axle |
| Wheelbase | `L` | 2.6 | m | `lf + lr` |
| Front cornering stiffness | `Cf` | 80 000 | N/rad | Per axle (linear range) |
| Rear cornering stiffness | `Cr` | 100 000 | N/rad | Per axle (linear range) |
| Steering ratio | `ratio` | 15 | — | SWA / wheel angle |

> [!NOTE]
> `Cf < Cr` intentionally models an **understeering** vehicle (Kus > 0), which is the standard tuning for passenger cars.  
> Swapping to `Cf > Cr` would model an oversteering vehicle.

---

## KPI Reference

### Handling KPIs (`computeHandlingKPIs`)

| KPI | Symbol | Unit | Definition |
|---|---|---|---|
| Peak Lateral Acceleration | `peakLatAccel_mps2` | m/s² | Max `abs(lateral_accel)` in maneuver window |
| Peak Yaw Rate | `peakYawRate_degps` | deg/s | Max `abs(yaw_rate)` in maneuver window |
| Yaw Rate Gain | `yawRateGain_1ps` | 1/s | `peakYawRate / peakSteer` |
| Lateral Acceleration Gain | `latAccelGain_mps2_per_deg` | m/s²/deg | `peakLatAccel / peakSteer` |
| Response Time | `responseTime_s` | s | Time to reach 90% of steady-state yaw rate |
| Settling Time | `settlingTime_s` | s | Time to stay within ±5% of final yaw rate |

### Steering KPIs (`computeSteeringKPIs`)

| KPI | Symbol | Unit | Definition |
|---|---|---|---|
| Steering Amplitude | `steeringAmplitude_deg` | deg | Max `abs(SWA)` in window |
| Peak Steering Rate | `peakSteeringRate_degps` | deg/s | Max `abs(dSWA/dt)` |
| Steer→Yaw Delay | `steeringToYawDelay_s` | s | Cross-correlation peak lag (SWA → Yaw Rate) |
| Steer→LatAcc Delay | `steeringToLatAccelDelay_s` | s | Cross-correlation peak lag (SWA → Lat. Acc.) |

### Ride KPIs (`computeRideKPIs`)

| KPI | Symbol | Unit | Definition |
|---|---|---|---|
| RMS Vertical Acceleration | `rmsVertAcc_mps2` | m/s² | Quadratic mean of `vertical_accel` |
| Peak-to-Peak Vertical Acc. | `peakToPeakVertAcc_mps2` | m/s² | `max - min` of `vertical_accel` |

### Frequency Response KPIs (`computeFrequencyResponse`)

| KPI | Symbol | Unit | Definition |
|---|---|---|---|
| Bandwidth (−3 dB) | `Bandwidth_Hz_Yaw/LatAcc` | Hz | Frequency where gain drops 3 dB from DC gain |
| Peak Gain | `PeakGain_Yaw/LatAcc` | — | Maximum amplitude of transfer function |
| Peak Frequency | `PeakFreq_Hz_Yaw/LatAcc` | Hz | Frequency at peak gain |
| Phase Lag @ 1 Hz | `PhaseLag_1Hz_deg_Yaw/LatAcc` | deg | Phase of TF at 1 Hz (standard handling metric) |

---

## Naming Conventions

| Element | Convention | Example |
|---|---|---|
| Function files | `camelCase` verb + noun | `computeHandlingKPIs.m` |
| KPI table columns | `camelCase` + unit suffix | `peakLatAccel_mps2` |
| Error IDs | `vdt:functionName:IssueType` | `vdt:loadVehicleData:FileNotFound` |
| Config fields | `section.parameterName_unit` | `cfg.lowPass.windowSamples` |

---

## Extending the Toolbox

To add a new KPI family (e.g., `computeLongitudinalKPIs`):

1. Create `src/core/computeLongitudinalKPIs.m` following the signature `kpis = computeXxxKPIs(data, events, cfg)`.
2. Return a `1×N` MATLAB table with descriptive column names and unit suffixes.
3. Call the new function inside `computeAllKPIs.m` and merge its columns using the existing `setdiff` pattern.
4. Add at least two test methods in `tests/ToolboxTest.m`: one for the happy path and one for an edge case.
