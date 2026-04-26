# Vehicle Dynamics KPI Toolbox

![MATLAB CI](https://github.com/tuo-username/vehicle-dynamics-kpi-toolbox/actions/workflows/matlab-ci.yml/badge.svg)

A MATLAB-based toolbox for processing vehicle dynamics data, extracting Key Performance Indicators (KPIs), and generating reports.

## Features

- **Data Loading & Preprocessing**: Automated CSV loading, resampling, and low-pass filtering.
- **Maneuver Detection**: Automatic detection of steering events based on configurable thresholds.
- **KPI Extraction**:
  - **Handling**: Yaw rate gain, lateral acceleration gain, response time, settling time.
  - **Steering**: Steering rate, delays (Steer-to-Yaw, Steer-to-LatAcc).
  - **Ride**: RMS vertical acceleration, peak-to-peak acceleration.
- **Visualization**: Automated time-series plotting and handling characteristic plots.
- **Batch Processing**: Tools to process multiple datasets and export summaries.

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

## Project Status & Roadmap

### ✅ Current Status (MVP)
- **Modular Architecture**: Clean separation between IO, Preprocessing, and Core KPIs.
- **Bicycle Model Integration**: Theoretical ground truth comparison for yaw response.
- **Robustness**: Handling of noisy data, NaNs, and missing columns with specific Error IDs.
- **Testing**: Native MATLAB unit testing class (`ToolboxTest.m`).
- **Synthetic Data**: Integrated generator for Step, Sine, and Ride events.

### 🚀 Future Roadmap
- **Frequency Response Analysis**: Transfer functions (Gain/Phase) and Coherence.
- **Advanced Filtering**: Integration of Butterworth and zero-phase `filtfilt` algorithms.
- **ISO Standard Compliance**: Weighted vertical acceleration filters (ISO 2631).
- **Interactive UI**: MATLAB App Designer interface for drag-and-drop analysis.
- **PDF Reporting**: Automated generation of technical data sheets.

## Requirements

- **MATLAB**: R2022b or later recommended.
- **Toolboxes**: No additional toolboxes required (designed for maximum portability).
