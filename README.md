# Vehicle Dynamics KPI Toolbox

![MATLAB CI](https://github.com/cicixgliamici/vehicle-dynamics-kpi-toolbox/actions/workflows/matlab-ci.yml/badge.svg)

A MATLAB-based toolbox for processing vehicle dynamics data, extracting Key Performance Indicators (KPIs), and generating reports. Il toolbox integra modelli fisici avanzati (Bicycle Model) e analisi spettrale per una validazione scientifica dei risultati.

## Features

- **Data Loading & Preprocessing**: Automated CSV loading, resampling, and low-pass filtering.
- **Maneuver Detection**: Automatic detection of steering events based on configurable thresholds.
- **KPI Extraction**:
  - **Handling**: Yaw rate gain, lateral acceleration gain, response time, settling time.
  - **Steering**: Steering rate, transport delays via cross-correlation (Steer-to-Yaw, Steer-to-LatAcc).
  - **Ride**: RMS vertical acceleration, peak-to-peak acceleration.
- **Frequency Analysis**: Custom Welch PSD/CSD estimator (no toolbox), Bode gain/phase, coherence, bandwidth.
- **Visualization**: Automated time-series plotting and handling characteristic plots.
- **Batch Processing & Export**: Tools to process multiple datasets and export structured CSV summaries with traceability metadata.
- **Sideslip Estimation**: Linear Kalman Filter (2-state bicycle model) estimates the vehicle sideslip angle β — not measurable by standard sensors.

## Repository Structure

```text
vehicle-dynamics-kpi-toolbox/
├── src/                # Source code
│   ├── core/           # KPI computation logic
│   ├── io/             # Data loading and exporting
│   ├── maneuvers/      # Event detection algorithms
│   ├── prep/           # Signal preprocessing and filtering
│   └── viz/            # Plotting and visualization
├── scripts/            # Example scripts and utilities
├── tests/              # Unit tests
├── docs/               # Documentation
│   └── ARCHITECTURE.md # Data pipeline, KPI reference, naming conventions
└── startup.m           # Environment setup script
```

## Getting Started

1. Open MATLAB and navigate to the project root.
2. Run `startup.m` to set up the paths.
3. Generate synthetic data (if you don't have real data):
   ```matlab
   generateSyntheticData();
   ```
4. Run the demo analysis:
   ```matlab
   run('scripts/runSingleManeuverAnalysis.m');
   ```

## Testing & Validation

### Professional Unit Tests
Il toolbox usa il framework nativo di MATLAB. Per eseguire tutti i test:
```matlab
results = runtests('tests/ToolboxTest.m');
table(results) % Mostra i risultati in formato tabella
```

### Robustness Validation
Per verificare come il toolbox gestisce il rumore dei sensori:
```matlab
run('scripts/validateRobustness.m')
```
Questo script confronterà i dati "sporchi" filtrati con il valore teorico reale.

## Security & Reliability

- **Automated Pipeline**: Include linting statico, unit testing e auto-deployment di report PDF/CSV tramite GitHub Actions.
- **Defensive Programming**: Validazione dei tipi di dato (`isnumeric`) e gestione dei timestamp non monotoni per prevenire errori in produzione.
- **Large Scale Validation**: Sistema di generazione batch per testare la scalabilità su centinaia di dataset sintetici con vari livelli di rumore.

## Project Status & Roadmap

### 📐 Documentation

| Document | Description |
|---|---|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | Data pipeline, KPI reference, naming conventions, Kalman Filter |
| [MATH_REFERENCE.md](docs/MATH_REFERENCE.md) | Full mathematical derivations: filter frequency response, Welch PSD, cross-correlation delay, bicycle model, KF observability proof |

### ✅ Current Status (MVP)
- **Modular Architecture**: Clean separation between IO, Preprocessing, and Core KPIs.
- **Bicycle Model Integration**: Theoretical ground truth comparison for yaw response.
- **Sideslip Estimation**: Time-varying Linear Kalman Filter (2-state) for β estimation from yaw rate — with covariance tracking and innovation diagnostics.
- **Robustness**: Handling of noisy data, NaNs, and missing columns with specific Error IDs.
- **Testing**: Native MATLAB unit testing class (`ToolboxTest.m`) — 14 tests covering all modules including KF stability and physics checks.
- **Synthetic Data**: Integrated generator for Step, Sine, Ride, and Frequency Sweep events.

### 🚀 Future Roadmap
- **Advanced Filtering**: Butterworth / zero-phase `filtfilt` IIR filter (requires Signal Processing Toolbox).
- **ISO 2631 Compliance**: Frequency-weighted RMS vertical acceleration for standardised ride comfort scoring.
- **Nonlinear Tyre Model**: Replace linear cornering stiffness with Pacejka Magic Formula for limit behaviour.
- **EKF Upgrade**: Extend the Kalman Filter to handle nonlinear dynamics (Pacejka tyres, varying Cf/Cr).
- **Interactive UI**: MATLAB App Designer interface for drag-and-drop analysis.
- **PDF Reporting**: Automated generation of technical data sheets via MATLAB Report Generator.

## Requirements

- **MATLAB**: R2022b or later recommended.
- **Toolboxes**: No additional toolboxes required (designed for maximum portability).
