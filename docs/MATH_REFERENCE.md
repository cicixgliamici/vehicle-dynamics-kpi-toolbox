# Mathematical & Algorithmic Reference

> This document provides the detailed mathematical derivations and algorithmic
> descriptions for every non-trivial computation in the Vehicle Dynamics KPI Toolbox.
> It is intended both as a standalone reference and as a companion to the source code.

---

## Table of Contents

1. [Signal Preprocessing Pipeline](#1-signal-preprocessing-pipeline)
   - 1.1 [Missing Sample Interpolation](#11-missing-sample-interpolation)
   - 1.2 [Uniform Resampling](#12-uniform-resampling)
   - 1.3 [Moving-Average Low-Pass Filter](#13-moving-average-low-pass-filter)
2. [Maneuver Detection](#2-maneuver-detection)
3. [Handling KPIs](#3-handling-kpis)
   - 3.1 [Gain Metrics](#31-gain-metrics)
   - 3.2 [Response Time & Settling Time](#32-response-time--settling-time)
4. [Steering KPIs](#4-steering-kpis)
   - 4.1 [Transport Delay via Cross-Correlation](#41-transport-delay-via-cross-correlation)
5. [Frequency Response Analysis (Welch Method)](#5-frequency-response-analysis-welch-method)
   - 5.1 [Power Spectral Density Estimation](#51-power-spectral-density-estimation)
   - 5.2 [Transfer Function and Coherence](#52-transfer-function-and-coherence)
   - 5.3 [Scalar KPIs from the Frequency Domain](#53-scalar-kpis-from-the-frequency-domain)
6. [Bicycle Model — Steady-State](#6-bicycle-model--steady-state)
7. [Kalman Filter — Sideslip Estimation](#7-kalman-filter--sideslip-estimation)
   - 7.1 [State-Space Formulation](#71-state-space-formulation)
   - 7.2 [Observability Analysis](#72-observability-analysis)
   - 7.3 [Discretisation](#73-discretisation)
   - 7.4 [Kalman Recursion](#74-kalman-recursion)
   - 7.5 [Tuning Guidelines](#75-tuning-guidelines)

---

## 1. Signal Preprocessing Pipeline

### 1.1 Missing Sample Interpolation

**File:** [`src/prep/removeMissingSamples.m`](../src/prep/removeMissingSamples.m)

Vehicle telemetry often contains **dropout gaps** — intervals where sensors fail
to transmit data, resulting in `NaN` values. The strategy used here is
**piecewise linear interpolation**:

For each gap between two valid samples at times $t_a$ and $t_b$, every missing
sample at $t_k \in (t_a, t_b)$ is filled by:

$$x(t_k) = x(t_a) + \frac{x(t_b) - x(t_a)}{t_b - t_a} \cdot (t_k - t_a)$$

For dropouts at the **boundaries** of the recording (before the first valid
sample or after the last), nearest-neighbour extrapolation is used to avoid
introducing phantom trends:

$$x(t_k < t_{\text{first}}) = x(t_{\text{first}}), \quad x(t_k > t_{\text{last}}) = x(t_{\text{last}})$$

> [!NOTE]
> Linear interpolation is appropriate for slowly varying physical signals (yaw rate,
> lateral acceleration). For high-frequency or highly non-linear signals, more
> sophisticated gap-filling (spline, Akima) would be preferable.

---

### 1.2 Uniform Resampling

**File:** [`src/prep/resampleVehicleData.m`](../src/prep/resampleVehicleData.m)

Real vehicle loggers often produce **non-uniform time steps** due to varying CAN
bus load or asynchronous sensor triggers. Frequency-domain analysis (Section 5)
requires a uniform sampling grid. The procedure:

1. Construct a new, strictly uniform time vector:

$$t^*_k = t_{\text{start}} + k \cdot \Delta t^*, \quad k = 0, 1, \ldots, N-1$$

where $\Delta t^* = $ `cfg.targetSampleTime_s` (default: 0.01 s → 100 Hz).

2. For each physical channel $x(t)$, compute the values on the new grid via
   **linear interpolation**:

$$x(t^*_k) = \text{interp1}(t,\, x,\, t^*_k,\, \text{`linear'})$$

**Why linear interpolation?** It is the minimum-assumption interpolant that
avoids ringing artefacts (Gibbs phenomenon) associated with sinc interpolation,
and it is well-conditioned even for signals with sharp transitions (step steers).

---

### 1.3 Moving-Average Low-Pass Filter

**File:** [`src/prep/lowPassFilterSignal.m`](../src/prep/lowPassFilterSignal.m)

A **symmetric (zero-phase) moving-average** filter of length $M$ (always odd)
computes each output sample as:

$$y[n] = \frac{1}{M} \sum_{k = -(M-1)/2}^{(M-1)/2} x[n + k]$$

**Frequency response.** The transfer function $H(e^{j\omega})$ of a length-$M$
moving average is the *Dirichlet kernel*:

$$|H(e^{j\omega})| = \frac{1}{M} \left| \frac{\sin(M\omega/2)}{\sin(\omega/2)} \right|$$

The **−3 dB cutoff frequency** (in normalised units) is approximately:

$$f_c \approx \frac{0.443}{M} \cdot f_s$$

For $M = 9$ and $f_s = 100$ Hz: $f_c \approx 4.9$ Hz, which attenuates sensor
noise above ~5 Hz while preserving the dynamic content of steering manoeuvres
(typically 0–3 Hz).

**Zero-phase property.** Because the window is centred symmetrically around $n$,
the filter introduces **no phase delay**. This is critical for delay estimation
(Section 4.1): post-filtering, the relative timing between steering and yaw rate
is preserved.

> [!IMPORTANT]
> `MATLAB`'s `movmean(..., 'omitnan')` option gracefully handles isolated residual
> `NaN` values by averaging the available neighbours — removing the need for a
> second pass through `removeMissingSamples`.

---

## 2. Maneuver Detection

**File:** [`src/maneuvers/detectManeuvers.m`](../src/maneuvers/detectManeuvers.m)

The detector uses a **threshold hysteresis trigger** on the absolute steering
wheel angle:

```
ACTIVE ← |δ_sw(t)| > θ_threshold
```

where `cfg.maneuver.steeringThreshold_deg` = 1.0 deg (configurable).

**State machine logic** (implemented with `diff`):

| Condition | Event |
|---|---|
| `ACTIVE[k-1] = 0, ACTIVE[k] = 1` | **Rising edge** → manoeuvre start |
| `ACTIVE[k-1] = 1, ACTIVE[k] = 0` | **Falling edge** → manoeuvre end |

**Duration filter.** Short pulses below `cfg.maneuver.minDuration_s` (0.5 s)
are discarded to eliminate noise-induced false triggers:

$$\text{Valid} \iff t_{\text{end}} - t_{\text{start}} \geq 0.5 \text{ s}$$

**Fallback.** If no manoeuvre is detected (e.g. constant-radius test at small
steering angle), the entire recording is treated as a single event. This prevents
the KPI pipeline from returning empty results.

---

## 3. Handling KPIs

**File:** [`src/core/computeHandlingKPIs.m`](../src/core/computeHandlingKPIs.m)

All metrics are extracted from the **first detected event window**
$[t_{\text{start}},\ t_{\text{end}}]$.

### 3.1 Gain Metrics

Let $\hat{\delta}$, $\hat{r}$, $\hat{a}_y$ denote the peak absolute values of
steering wheel angle, yaw rate, and lateral acceleration within the event window.

**Yaw Rate Gain** (dimensionless, units: $\text{s}^{-1}$ after SWA normalisation):

$$G_r = \frac{\hat{r}}{\hat{\delta}}$$

**Lateral Acceleration Gain** (units: $\text{m/s}^2/\text{deg}$):

$$G_{a_y} = \frac{\hat{a}_y}{\hat{\delta}}$$

Both use peak-to-peak approximation (valid for step and quasi-static manoeuvres).
For dynamic manoeuvres, the frequency-domain gains (Section 5) are more
appropriate.

> [!NOTE]
> **Safe division** is enforced: if $|\hat{\delta}| < \varepsilon$ (machine epsilon),
> the gain is returned as `NaN` to prevent division-by-zero artefacts.

---

### 3.2 Response Time & Settling Time

These metrics characterise the **transient** of the yaw rate response to a
steering input.

#### Response Time $t_r$

The time from the start of the manoeuvre to when the yaw rate first reaches a
fraction $p$ of its final (steady-state) value:

$$t_r = \min \left\{ t \,\middle|\, |r(t)| \geq p \cdot |r_{\infty}| \right\} - t_{\text{start}}$$

Default: $p = 0.90$ (`cfg.kpi.responsePercentage`). This is the standard
**90% rise time** criterion.

#### Settling Time $t_s$

The time at which the yaw rate enters and **stays** within a ±band around the
final value:

$$t_s = \min \left\{ t_k \,\middle|\, \forall\, j \geq k:\; |r(t_j) - r_{\infty}| \leq \eta \cdot |r_{\infty}| \right\} - t_{\text{start}}$$

Default: $\eta = 0.05$ (5% band, `cfg.kpi.settlingBandPercentage`).

The **steady-state reference** $r_{\infty}$ is computed as the mean of the final
`cfg.kpi.steadyStateWindow_s` seconds of the event window (default: last 1 s).

---

## 4. Steering KPIs

**File:** [`src/core/computeSteeringKPIs.m`](../src/core/computeSteeringKPIs.m)

### 4.1 Transport Delay via Cross-Correlation

The **transport delay** $\tau$ between steering input $u[n]$ and vehicle response
$y[n]$ (yaw rate or lateral acceleration) is estimated using normalised
**cross-correlation**:

$$R_{uy}[\ell] = \sum_{n=0}^{N-1-|\ell|} \tilde{u}[n] \cdot \tilde{y}[n+\ell]$$

where $\tilde{u}$, $\tilde{y}$ are the zero-mean, unit-variance normalised signals:

$$\tilde{x}[n] = \frac{x[n] - \mu_x}{\sigma_x + \varepsilon}$$

The estimated delay is the lag that maximises the cross-correlation:

$$\hat{\tau} = \ell^* \cdot \Delta t, \quad \ell^* = \arg\max_{\ell \in [-L,\, L]} R_{uy}[\ell]$$

The search window is capped at $L = \lfloor 1\text{ s} / \Delta t \rfloor$ samples
(1 second maximum physiological delay for a road vehicle).

**Why not use derivative peaks?**
The previous approach (compare times of peak absolute derivative of $u$ and peak
absolute value of $y$) is highly sensitive to noise and signal shape. A sine
sweep, for example, has no single peak derivative — but the cross-correlation
correctly identifies the phase shift at all frequencies simultaneously.

**Why normalise?**
Without normalisation, $R_{uy}$ depends on signal amplitude. A large steering
input would dominate the correlation, biasing the lag estimate. Normalisation
makes the metric dimensionless and scale-invariant.

---

## 5. Frequency Response Analysis (Welch Method)

**File:** [`src/core/computeFrequencyResponse.m`](../src/core/computeFrequencyResponse.m)

Classical Welch's method estimates **Power Spectral Densities (PSDs)** and
**Cross-Spectral Densities (CSDs)** by averaging short-time DFTs over
overlapping segments. This reduces variance at the cost of frequency resolution.

### 5.1 Power Spectral Density Estimation

**Step 1 — Detrend.** Remove linear trends from each signal to eliminate DC
offset and low-frequency drift that would bias the PSD:

$$\tilde{x}[n] = x[n] - (a_0 + a_1 n), \quad [a_0, a_1] = \text{polyfit}(n,\, x,\, 1)$$

**Step 2 — Segment and window.** Divide the signal into $K$ overlapping
segments of length $L$ with step $L - \text{overlap}$:

$$x_k[m] = \tilde{x}[k \cdot \text{step} + m] \cdot w[m], \quad m = 0, \ldots, L-1$$

where $w[m]$ is the **Hann window** (implemented from scratch, no toolbox):

$$w[m] = 0.5 \left(1 - \cos\!\left(\frac{2\pi m}{L-1}\right)\right)$$

The Hann window provides −31.5 dB sidelobe attenuation, preventing spectral
leakage from high-amplitude tones from contaminating adjacent bins.

**Step 3 — DFT per segment.** For the input channel $U$ and output channel $Y$:

$$U_k[q] = \text{FFT}(x_k)[q], \quad q = 0, \ldots, L/2$$

Only the **positive half** of the spectrum is retained (real signals have
Hermitian symmetry).

**Step 4 — Accumulate and average.** Auto- and cross-spectra are accumulated
across $K$ segments then divided:

$$\hat{S}_{uu}[q] = \frac{1}{K} \sum_{k=0}^{K-1} |U_k[q]|^2, \quad
  \hat{S}_{uy}[q] = \frac{1}{K} \sum_{k=0}^{K-1} U_k^*[q] \cdot Y_k[q]$$

Averaging over $K$ segments reduces the **variance** of the PSD estimate by a
factor of $K$ compared to a single periodogram.

---

### 5.2 Transfer Function and Coherence

**Transfer Function** (frequency-domain gain and phase from input $u$ to output $y$):

$$\hat{H}_{uy}[q] = \frac{\hat{S}_{uy}[q]}{\hat{S}_{uu}[q]}$$

This is the **H1 estimator**, optimal when noise is present only on the output.

**Magnitude (Bode gain):** $|H[q]|$

**Phase:** $\angle H[q] = \text{unwrap}(\arg H[q])$, unwrapped to remove ±180°
jumps and show the continuous phase lag.

**Coherence** (measure of linear correlation between $u$ and $y$ at each frequency):

$$\gamma^2_{uy}[q] = \frac{|\hat{S}_{uy}[q]|^2}{\hat{S}_{uu}[q] \cdot \hat{S}_{yy}[q]} \in [0,\, 1]$$

$\gamma^2 = 1$: perfect linear relationship at that frequency (pure signal).
$\gamma^2 \approx 0$: no linear relationship (noise dominated or nonlinear).

---

### 5.3 Scalar KPIs from the Frequency Domain

| KPI | Formula | Interpretation |
|---|---|---|
| **Bandwidth** | $f_{-3\text{dB}} = \min\{f : \|H(f)\| < \|H(f_0)\| / \sqrt{2}\}$ | Vehicle's usable frequency range |
| **Peak Gain** | $\max_f \|H(f)\|$ | Resonance amplitude |
| **Peak Frequency** | $\arg\max_f \|H(f)\|$ | Resonance frequency |
| **Phase Lag @ 1 Hz** | $\angle H(1\text{ Hz})$ | Standard handling metric (ISO target: < −30°) |

---

## 6. Bicycle Model — Steady-State

**File:** [`src/core/simulateBicycleModel.m`](../src/core/simulateBicycleModel.m)

The **linear bicycle model** is the standard reduced-order model for vehicle
lateral dynamics. It assumes:
- Small sideslip angles (linear tyre range)
- Constant speed $v$
- Negligible roll and pitch dynamics

**Tyre slip angles** (front $\alpha_f$, rear $\alpha_r$):

$$\alpha_f = \delta_f - \beta - \frac{l_f r}{v}, \quad \alpha_r = -\beta + \frac{l_r r}{v}$$

**Lateral tyre forces** (linear Pacejka approximation):

$$F_{yf} = C_f \alpha_f, \quad F_{yr} = C_r \alpha_r$$

**Equations of motion:**

$$m(\dot{v}_y + v_x r) = C_f\alpha_f + C_r\alpha_r$$
$$I_z \dot{r} = l_f C_f\alpha_f - l_r C_r\alpha_r$$

**Understeer Gradient** (steady-state, $\dot{\beta} = \dot{r} = 0$):

$$K_{us} = \frac{m}{L}\left(\frac{l_r}{C_f} - \frac{l_f}{C_r}\right) \quad [\text{rad/(m/s}^2)]$$

- $K_{us} > 0$ → **understeer** (standard passenger car): at higher speed, more
  steering angle is needed to maintain the same radius.
- $K_{us} < 0$ → **oversteer**: vehicle tends to spin.

**Steady-State Yaw Rate Gain** (per unit wheel angle):

$$\frac{r}{\delta_f}\bigg|_{\text{ss}} = \frac{v}{L + K_{us} v^2}$$

---

## 7. Kalman Filter — Sideslip Estimation

**File:** [`src/core/estimateSideslip.m`](../src/core/estimateSideslip.m)

> See also the [Architecture Guide](./ARCHITECTURE.md) for a summary table of the
> filter equations.

### 7.1 State-Space Formulation

**State vector:**

$$\mathbf{x} = \begin{bmatrix} \beta \\ r \end{bmatrix}$$

where $\beta$ [rad] is the **vehicle sideslip angle** (unmeasurable) and $r$ [rad/s]
is the yaw rate (measured by IMU).

**Continuous-time plant** (derived from the bicycle model equations in Section 6,
evaluated at instantaneous speed $v_k$):

$$\dot{\mathbf{x}} = A(v)\,\mathbf{x} + B(v)\,u + \mathbf{w}$$

$$A(v) = \begin{bmatrix}
  -\dfrac{C_f+C_r}{mv}  &  \dfrac{C_r l_r - C_f l_f}{mv^2} - 1 \\[10pt]
  \dfrac{C_r l_r - C_f l_f}{I_z}  &  -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v}
\end{bmatrix}, \quad
B(v) = \begin{bmatrix} C_f/(mv) \\ C_f l_f/I_z \end{bmatrix}$$

where $u = \delta_f$ [rad] is the front wheel angle (steering wheel angle divided
by the steering ratio) and $\mathbf{w} \sim \mathcal{N}(\mathbf{0}, Q)$ is
process noise (model uncertainty).

**Measurement equation:**

$$y_k = C\,\mathbf{x}_k + v_k, \quad C = \begin{bmatrix} 0 & 1 \end{bmatrix}, \quad v_k \sim \mathcal{N}(0, R)$$

The IMU measures yaw rate $r$ directly; the sideslip $\beta$ is the **hidden state**.

---

### 7.2 Observability Analysis

The system is **completely observable** if and only if the observability matrix
has full rank:

$$\mathcal{O} = \begin{bmatrix} C \\ CA \end{bmatrix} = \begin{bmatrix}
  0 & 1 \\[4pt]
  \dfrac{C_r l_r - C_f l_f}{I_z} & -\dfrac{C_f l_f^2 + C_r l_r^2}{I_z v}
\end{bmatrix}$$

$$\det(\mathcal{O}) = -\frac{C_r l_r - C_f l_f}{I_z} \neq 0 \quad \text{for } C_r l_r \neq C_f l_f$$

Since a real vehicle always has different front/rear moment arms
($C_f l_f \neq C_r l_r$), $\text{rank}(\mathcal{O}) = 2$ and the sideslip
$\beta$ is **fully recoverable from yaw rate measurements alone**.

Physical interpretation: knowing $r(t)$ and the input $\delta_f(t)$, the
observer can back-calculate the forces required to produce that yaw rate and,
from the force balance, infer what $\beta$ must be.

---

### 7.3 Discretisation

At each time step $k$, the continuous-time matrices are **Euler-discretised**:

$$\Phi_k = I_2 + A(v_k)\,\Delta t, \quad \Gamma_k = B(v_k)\,\Delta t$$

This is a **time-varying** discretisation: $\Phi_k$ and $\Gamma_k$ are
recomputed at every sample using the current speed $v_k$. This makes the filter
more accurate during acceleration/braking phases compared to a fixed-speed
approximation.

**Stability condition** (Von Neumann criterion for Euler method): the Euler
scheme is stable if all eigenvalues $\lambda_i$ of $A$ satisfy
$|1 + \lambda_i \Delta t| < 1$.

For $\Delta t = 0.01$ s and typical vehicle parameters, all eigenvalues of $A$
have negative real parts (stable plant) with magnitudes much smaller than
$1/\Delta t = 100$, so Euler discretisation is stable at 100 Hz.

---

### 7.4 Kalman Recursion

The filter alternates between two steps at each sample $k$:

#### Predict Step (a priori)

Propagate the state and the error covariance using the dynamic model:

$$\hat{\mathbf{x}}^-_k = \Phi_k\,\hat{\mathbf{x}}_{k-1} + \Gamma_k\,u_k$$

$$P^-_k = \Phi_k\,P_{k-1}\,\Phi_k^\top + Q$$

The covariance update $P^- = \Phi P \Phi^\top + Q$ is the **discrete Riccati
equation** in its time-stepping form. $Q$ inflates $P$ at each step to model
growing uncertainty from unmodelled dynamics.

#### Update Step (a posteriori)

Correct the prediction using the actual measurement $y_k$:

**Innovation covariance** (scalar since $C \in \mathbb{R}^{1\times 2}$):

$$S_k = C\,P^-_k\,C^\top + R \in \mathbb{R}$$

**Kalman Gain** — the optimal weighting between prediction and measurement:

$$K_k = \frac{P^-_k\,C^\top}{S_k} \in \mathbb{R}^{2\times 1}$$

A large $K$ means "trust the measurement more"; a small $K$ means "trust the
model more." As the filter converges, $K$ stabilises at a steady-state value.

**State correction:**

$$\hat{\mathbf{x}}_k = \hat{\mathbf{x}}^-_k + K_k\,\underbrace{(y_k - C\,\hat{\mathbf{x}}^-_k)}_{\text{innovation } \nu_k}$$

The **innovation** $\nu_k = y_k - C\hat{\mathbf{x}}^-_k$ is the residual between
the measured yaw rate and the predicted yaw rate. A white-noise (zero-mean,
uncorrelated) innovation sequence confirms that the filter is **consistent** —
it has extracted all predictable information from the data.

**Covariance correction:**

$$P_k = (I_2 - K_k\,C)\,P^-_k$$

This update **decreases** $P$ (reduces uncertainty) when a measurement is
incorporated. The filter converges when $P$ stabilises (steady-state Riccati
solution).

---

### 7.5 Tuning Guidelines

The filter behaviour is controlled by three matrices:

| Parameter | Effect of increasing | Effect of decreasing |
|---|---|---|
| $Q$ (process noise) | More responsive to measurements; β can change faster | Smoother β; slow to react to sudden changes |
| $R$ (measurement noise) | More weight on the model prediction | More weight on yaw rate sensor |
| $P_0$ (initial covariance) | Faster initial convergence; aggressive first few steps | Conservative start; β initialised close to 0 |

**Diagnostic check — Innovation whiteness test:**
If the innovations $\{\nu_k\}$ are autocorrelated, the filter is not capturing
all available information. Common causes:
- $Q$ too small (model trusted too much, unmodelled dynamics ignored)
- $R$ too large (measurements under-weighted)

Compute the normalised innovation squared (NIS):

$$\text{NIS}_k = \frac{\nu_k^2}{S_k}$$

For a consistent filter, $\text{NIS}_k \sim \chi^2(1)$, i.e. its mean should be
close to 1.0 over a long run.

---

*Document maintained alongside the source code. If an algorithm changes, update
the corresponding section here. Last structural update: July 2026.*
