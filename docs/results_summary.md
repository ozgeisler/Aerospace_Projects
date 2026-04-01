# Results Summary — UAV FDA Disturbance Sensitivity

> **Auto-fill this file** after running `main_run.m` and `export_report.m`.

---

## 1. Model Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Total mass | m | 1.5 | kg |
| Roll inertia | Ixx | 0.0347 | kg·m² |
| Pitch inertia | Iyy | 0.0458 | kg·m² |
| Yaw inertia | Izz | 0.0977 | kg·m² |
| Arm length | l | 0.225 | m |
| Thrust coeff | kF | 6.11×10⁻⁸ | N/(rad/s)² |
| Hover rotor speed | ω_h | ~382 | rad/s |

---

## 2. Linearization Results

**Trim point:** hover, all body velocities and rates = 0, T = m·g = 14.72 N

**Open-loop eigenvalues:**

| Channel | Poles | Type |
|---------|-------|------|
| Roll  | 0, 0 | Double integrator |
| Pitch | 0, 0 | Double integrator |
| Yaw   | 0, 0 | Double integrator |
| Heave | 0, 0 | Double integrator |

All channels are **marginally stable** open-loop (as expected for UAV attitude dynamics).

---

## 3. Stability Margins

*(Fill after running `bode_analysis.m`)*

| Channel | GM [dB] | PM [°] | ωgc [rad/s] | ωpc [rad/s] | Status |
|---------|---------|--------|------------|------------|--------|
| Roll    | — | — | — | — | — |
| Pitch   | — | — | — | — | — |
| Yaw     | — | — | — | — | — |
| Heave   | — | — | — | — | — |

**Design requirement:** GM ≥ 6 dB, PM ≥ 30°

---

## 4. Sensitivity Analysis Results

*(Fill after running `sensitivity.m`)*

| Channel | ‖S‖∞ [dB] | Peak ω [rad/s] | ‖T‖∞ [dB] | BW [rad/s] |
|---------|-----------|---------------|-----------|-----------|
| Roll    | — | — | — | — |
| Pitch   | — | — | — | — |
| Yaw     | — | — | — | — |
| Heave   | — | — | — | — |

**Design requirement:** ‖S‖∞ ≤ 6 dB (factor of 2 — no disturbance amplification)

**Waterbed effect:** Bode's integral theorem enforces that sensitivity attenuation at low frequencies is compensated by amplification at higher frequencies. Trade-off is unavoidable.

---

## 5. PSD Analysis Results

*(Fill after running `psd_analysis.m`)*

### Dryden Turbulence Characteristics
| Component | σ [m/s] | L [m] | Dominant ω [rad/s] |
|-----------|---------|-------|-------------------|
| Longitudinal (u) | 1.5 | 200 | — |
| Lateral (v) | 1.5 | 200 | — |
| Vertical (w) | 0.75 | 50 | — |

### Disturbance Response Power
| Response | Total Power | Peak ω [rad/s] |
|----------|-------------|---------------|
| Heave z (to w-disturbance) | — | — |
| Roll φ (to u-disturbance) | — | — |

---

## 6. Key Findings

*(Fill after completing analysis)*

1. **Most sensitive channel:** TBD — the channel with highest ‖S‖∞
2. **Critical frequency band:** TBD — where disturbances are amplified
3. **Waterbed trade-off:** Low-frequency disturbance rejection comes at cost of high-frequency sensitivity peak
4. **Dryden turbulence:** Most energy concentrated below ~1 rad/s; heave channel most affected by vertical gusts

---

## 7. Generated Figures

| File | Description |
|------|-------------|
| `bode_openloop.png` | Open-loop Bode for all 4 channels |
| `stability_margins.png` | GM and PM bar chart |
| `nyquist.png` | Nyquist diagrams |
| `sensitivity_all.png` | S, T, PS per channel |
| `sensitivity_comparison.png` | ‖S‖∞ comparison across channels |
| `waterbed.png` | Waterbed effect illustration (roll) |
| `psd_disturbance.png` | Dryden turbulence PSD vs theory |
| `psd_response.png` | Closed-loop response PSD |
