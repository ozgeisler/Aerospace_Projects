# UAV Frequency Domain Analysis — Disturbance Sensitivity

**Frequency Domain Analysis of Disturbance Sensitivity in UAV Flight Dynamics**

Generic 6-DOF UAV modelinin frekans domeninde bozucu hassasiyeti analizi.
MATLAB/Simulink ile geliştirilmiş teorik simülasyon projesi.

---

## Overview

This project analyzes how a quadrotor UAV responds to atmospheric disturbances (wind turbulence) in the frequency domain. Starting from a full nonlinear 6-DOF model, we:

1. **Linearize** the dynamics at a hover trim point
2. **Compute** Bode plots and stability margins for all flight channels
3. **Analyze** sensitivity functions S(jω), T(jω), PS(jω)
4. **Estimate** disturbance power via Welch PSD (Dryden wind model)
5. **Export** a full HTML report with figures

---

## Repository Structure

```
UAV_FDA_Disturbance/
├── models/
│   ├── params.m            UAV physical parameters
│   ├── linear_model.m      Hover trim linearization → A,B,C,D, TF channels
│   └── uav_6dof.slx        Simulink nonlinear model (optional)
├── analysis/
│   ├── bode_analysis.m     Bode / Nyquist / stability margins
│   ├── sensitivity.m       S, T, PS sensitivity functions
│   └── psd_analysis.m      Welch PSD, Dryden turbulence simulation
├── results/
│   ├── figures/            Generated PNG plots
│   └── data/               .mat result files
├── utils/
│   ├── plot_bode_custom.m  Publication-quality Bode plot function
│   └── export_report.m     Auto HTML report generator
├── docs/
│   ├── theory.md           Mathematical background
│   └── results_summary.md  Results template
├── main_run.m              Master pipeline script
└── README.md
```

---

## Requirements

| Tool | Version |
|------|---------|
| MATLAB | R2022a or later |
| Simulink | R2022a or later |
| Control System Toolbox | Required |
| Signal Processing Toolbox | Required |

---

## Quick Start

```matlab
% Clone / download the repo, then in MATLAB:
cd UAV_FDA_Disturbance
run('main_run.m')
```

This executes the full pipeline (params → linearization → Bode → sensitivity → PSD → report) and saves all figures to `results/figures/`.

### Run individual modules

```matlab
cd models
run('params.m')         % Load parameters
run('linear_model.m')   % Linearize

cd ../analysis
run('bode_analysis.m')  % Bode plots
run('sensitivity.m')    % Sensitivity functions
run('psd_analysis.m')   % PSD analysis

cd ../utils
export_report()         % Generate HTML report
```

---

## Key Concepts

### Sensitivity Function
```
S(jω) = 1 / (1 + G(jω)·C(jω))
```
Peak value ‖S‖∞ quantifies worst-case disturbance amplification.
**Design target:** ‖S‖∞ ≤ 6 dB (factor of 2).

### Waterbed Effect (Bode's Integral Theorem)
```
∫₀^∞ log|S(jω)| dω = 0
```
Sensitivity reduction at low frequencies is always traded for amplification at higher frequencies.

### Dryden Wind Model
Atmospheric turbulence is modeled as colored noise with power spectral density:
```
Φu(ω) = σu² · (2Lu/πV) / [1 + (Lu·ω/V)²]
```

---

## Channels Analyzed

| Channel | Plant G(s) | Controller C(s) |
|---------|-----------|-----------------|
| Roll  | 1/(Ixx·s²) | PD (Kp=8, Kd=3) |
| Pitch | 1/(Iyy·s²) | PD (Kp=8, Kd=3) |
| Yaw   | 1/(Izz·s²) | PD (Kp=3, Kd=1) |
| Heave | -1/(m·s²)  | PI (Kp=5, Ki=2) |

---

## References

- Stevens & Lewis (2003). *Aircraft Control and Simulation*
- Skogestad & Postlethwaite (2005). *Multivariable Feedback Design*
- MIL-SPEC-8785C (1980). Dryden Turbulence Model
- Bode (1945). Sensitivity integral theorem

---


