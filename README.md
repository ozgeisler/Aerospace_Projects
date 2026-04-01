# F-14 Carrier Launch Dynamics

## Model Overview
The aircraft is modeled as a rigid mass with aerodynamic drag:

m v_dot = u - c v

Lift is computed as:

L = 0.5 * rho * V^2 * C_L * S

## Scenarios
- Throttle and catapult activated simultaneously
- Engines spool up before catapult activation

## Key Parameters
- Weight: 230,000 N
- Drag coefficient: 100 Ns/m
- Air density: 1.225 kg/m^3
- Wing area: 54.5 m^2
- Lift coefficient: 1.4

## Results
Simulation evaluates whether the aircraft reaches lift-off speed within 91.44 meters of deck length.
