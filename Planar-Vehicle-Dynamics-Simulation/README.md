# Planar-Vehicle-Dynamics-Simulation

A from-scratch implementation of a 2D planar vehicle dynamics model in MATLAB and Simulink, based on classical control theory principles.

---


This project models the equations of motion of a planar vehicle with rotational thrusters. The model simulates translational and rotational dynamics using Newton's second law, and visualizes the vehicle's trajectory and states over time.

Built entirely from scratch — no pre-built library blocks used.

---



### State Vector

```
x = [x, y, ẋ, ẏ, θ, θ̇]
```

| State | Description |
|-------|-------------|
| x, y | Position (m) |
| ẋ, ẏ | Velocity (m/s) |
| θ | Heading angle (rad) |
| θ̇ | Angular rate (rad/s) |

### Equations of Motion

```
ẍ = (Fu·cos(θ) - Fd·cos(χ)) / m
ÿ = (Fu·sin(θ) - Fd·sin(χ)) / m
θ̈ = (Mu - CR·θ̇) / Ib
```

Where:
- `Fu` = thrust force, `Mu` = thrust moment
- `Fd = CT · V²` = translational drag
- `χ = atan2(ẏ, ẋ)` = course angle

---

## Vehicle Parameters 

| Parameter | Value | Description |
|-----------|-------|-------------|
| m | 10 kg | Mass |
| Ib | 5 kg·m² | Moment of inertia |
| Fmax | 10 N | Max thrust force |
| Mmax | 0.5 N·m | Max thrust moment |
| CT | 0.025 N·s²/m² | Translational drag coefficient |
| CR | 0.75 N·m·s | Rotational drag coefficient |

---



## ▶️ How to Run

1. Open MATLAB and set the current folder to this project directory.

2. Run the main script:
```matlab
run run_eom.m
```

3. The script will:
   - Load vehicle constants
   - Open and simulate the Simulink model
   - Plot trajectory and state outputs

---

##  Output

The simulation produces two figures:

**Figure 1 — Overall Trajectory**
- Shows the x-y path of the boat from start (○) to end (×)

**Figure 2 — Inputs & Outputs**
- `uAxial`, `uMoment` — throttle inputs
- `V` — vehicle speed over time
- `θ` — heading angle over time
- `dθ/dt` — angular rate over time

---



Implemented independently as a hands-on exercise in dynamic systems modeling and Simulink development.
