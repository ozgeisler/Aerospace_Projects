
## Linear vs Nonlinear Pendulum + Damping Identification (MATLAB/Simulink)

### Overview
This repository compares a **linearized pendulum** model and a **full nonlinear pendulum** model under identical initial conditions.
In addition, it estimates the damping ratio **ζ** and damping coefficient **b** from the simulated response using the **logarithmic decrement** method, then compares measured vs. actual parameters.

### Models
Nonlinear equation of motion:
J θ¨ = -m g L sin(θ) - b θ˙ + T(t)

Linearized (small-angle) model:
sin(θ) ≈ θ


- MATLAB script to run the Simulink model and extract outputs
- Linear vs nonlinear response comparison (θ vs time)
- Peak detection and **log decrement** damping estimation:
  - ζ_measured from η = θ2 / θ1
  - b_measured computed from ζ_measured
