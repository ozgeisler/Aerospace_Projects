# Theory — UAV Frequency Domain Analysis

## 1. Generic 6-DOF Rigid Body Dynamics

The UAV is modeled as a rigid body with 12 states:

**State vector:**
```
x = [u, v, w, p, q, r, φ, θ, ψ, x_E, y_E, z_E]ᵀ
```
- `u, v, w` — body-frame translational velocities [m/s]
- `p, q, r` — body-frame angular rates [rad/s]
- `φ, θ, ψ` — Euler angles (roll, pitch, yaw) [rad]
- `xE, yE, zE` — inertial position [m]

### 1.1 Translational Dynamics (Newton's 2nd Law)

```
m(v̇ + ω × v) = F_aero + F_gravity + F_disturbance
```

In body frame:
```
m·u̇ = m(rv - qw) - m·g·sinθ        + Fx_dist
m·v̇ = m(pw - ru) + m·g·cosθ·sinφ   + Fy_dist
m·ẇ = m(qu - pv) + m·g·cosθ·cosφ   - T + Fz_dist
```

### 1.2 Rotational Dynamics (Euler's Equations)

```
I·ω̇ + ω × (I·ω) = M_control + M_disturbance
```

```
Ixx·ṗ - (Iyy - Izz)·q·r = τφ
Iyy·q̇ - (Izz - Ixx)·p·r = τθ
Izz·ṙ - (Ixx - Iyy)·p·q = τψ
```

### 1.3 Kinematic Equations (Euler angle rates)

For small angles (hover approximation):
```
φ̇ ≈ p,  θ̇ ≈ q,  ψ̇ ≈ r
```

---

## 2. Linearization at Hover Trim

### 2.1 Trim Conditions
At hover: `u = v = w = p = q = r = 0`, `φ = θ = 0`, thrust `T = m·g`.

### 2.2 Jacobian Linearization

Expand the nonlinear dynamics `ẋ = f(x, u)` around trim `(x₀, u₀)`:

```
δẋ = A·δx + B·δu
δy = C·δx + D·δu
```

where:
```
A = ∂f/∂x |_(x₀,u₀),   B = ∂f/∂u |_(x₀,u₀)
```

### 2.3 Decoupled Channels

At hover, the linearized system decouples into four independent SISO channels:

| Channel | Input | Output | Transfer Function |
|---------|-------|--------|-------------------|
| Roll  | τφ | φ | G_roll(s) = 1/(Ixx·s²) |
| Pitch | τθ | θ | G_pitch(s) = 1/(Iyy·s²) |
| Yaw   | τψ | ψ | G_yaw(s) = 1/(Izz·s²) |
| Heave | T  | z | G_heave(s) = −1/(m·s²) |

All four are **double integrators** — marginally stable open-loop.

---

## 3. Frequency Domain Analysis

### 3.1 Open-Loop Transfer Function

```
L(s) = C(s) · G(s)
```

The **Bode plot** of L(jω) gives:
- **Gain crossover frequency** ωgc: |L(jωgc)| = 0 dB
- **Phase margin** PM = 180° + ∠L(jωgc)
- **Phase crossover frequency** ωpc: ∠L(jωpc) = −180°
- **Gain margin** GM = −20·log|L(jωpc)| [dB]

**Design targets:** GM ≥ 6 dB, PM ≥ 30°

### 3.2 Sensitivity Functions

For closed-loop `y = G·C·(r − y) + G·d`:

| Function | Formula | Physical meaning |
|----------|---------|-----------------|
| S(s) = 1/(1+L) | Sensitivity | How much disturbance/error appears at output |
| T(s) = L/(1+L) | Complementary sensitivity | Reference tracking |
| PS(s) = G/(1+L) | Input sensitivity | Effect of plant-input disturbance |

**Constraint:** `S(jω) + T(jω) = 1` for all ω

### 3.3 H∞ Norm and Disturbance Amplification

```
||S||∞ = max_ω |S(jω)| = sup_ω 1/|1 + L(jω)|
```

- `||S||∞ ≤ 2` (6 dB): good disturbance rejection
- `||S||∞ > 2`: system amplifies disturbances at peak frequency

### 3.4 Bode's Integral Theorem (Waterbed Effect)

For open-loop stable systems with relative degree ≥ 2:

```
∫₀^∞ log|S(jω)| dω = 0
```

**Implication:** Reducing sensitivity at low frequencies *must* increase it at higher frequencies. There is a fundamental trade-off — no free lunch in control design.

---

## 4. Disturbance Modeling — Dryden Turbulence

The Dryden wind turbulence model (MIL-SPEC-8785C) approximates atmospheric turbulence as a stationary Gaussian process:

### 4.1 PSD Formulas

**Longitudinal turbulence:**
```
Φu(ω) = σu² · (2Lu/πV) · 1 / [1 + (Lu·ω/V)²]
```

**Vertical turbulence:**
```
Φw(ω) = σw² · (Lw/πV) · 1 / [1 + (Lw·ω/V)²]
```

where `σ` = intensity [m/s], `L` = scale length [m], `V` = airspeed [m/s].

### 4.2 Shaping Filter (White Noise → Colored Turbulence)

```
H_u(s) = σu · √(2Lu/πV) / (1 + s·Lu/V)
```

Pass unit-variance white noise through H(s) to obtain Dryden turbulence.

### 4.3 Power Spectral Density via Welch Method

The Welch estimator splits signal into overlapping segments, applies a window (Hann), computes periodogram per segment, and averages:

```
P̂(f) = (1/KU) · Σₖ |Σₙ x_k[n]·w[n]·e^{-j2πfn}|²
```

where K = number of segments, U = window normalization factor.

---

## 5. PID Controller Structure

A filtered derivative PID controller is used for each channel:

```
C(s) = Kp + Ki/s + Kd · N·s/(s+N)
```

| Channel | Kp | Ki | Kd | N  |
|---------|----|----|----|----|
| Roll    | 8  | 0  | 3  | 50 |
| Pitch   | 8  | 0  | 3  | 50 |
| Yaw     | 3  | 0  | 1  | 30 |
| Heave   | 5  | 2  | 0  | —  |

---

## References

1. Stevens, B.L., Lewis, F.L. (2003). *Aircraft Control and Simulation*. Wiley.
2. Skogestad, S., Postlethwaite, I. (2005). *Multivariable Feedback Design*. Wiley.
3. MIL-SPEC-8785C (1980). *Flying Qualities of Piloted Airplanes*.
4. Welch, P.D. (1967). The use of fast Fourier transform for the estimation of power spectra. *IEEE Trans. Audio Electroacoust.*
5. Bode, H.W. (1945). *Network Analysis and Feedback Amplifier Design*. Van Nostrand.
