# DC Motor g11 (ω/Va) — State-Space ↔ Transfer Function Consistency + Canonical Realizations

## Goal
Extract and validate the SISO transfer function **g11(s) = ω(s) / Va(s)** from a DC motor MIMO state-space model using multiple equivalent approaches:
1) Hand-derived transfer function from the motor equations  
2) MATLAB `ss2tf` extraction from the original MIMO state-space (selecting input/output channel)  
3) Transfer function → controllable canonical-form state-space realization  
4) MATLAB `tf2ss` realization

## System Definition (MIMO State-Space)
State-space model:
\[
\dot{x} = Ax + Bu,\quad y = Cx + Du
\]
Channel of interest:
- Input: **u1 = Va (armature voltage)**
- Output: **y1 = ω = θ̇ (angular velocity)**

## Methods
### 1) Hand-derived transfer function
Derived:
\[
G_{11}(s)=\frac{ω(s)}{V_a(s)}=\frac{K_T}{J_mL_as^2 + (cL_a + J_m(R+R_m))s + (K_TK_V + c(R+R_m))}
\]
See: `derivation/`

### 2) Extract TF from MIMO SS using `ss2tf`
`ss2tf(A, B(:,1), C(1,:), D(1,1))`

### 3) TF → controllable canonical-form SS
Monic normalization and canonical realization were constructed and validated by converting back to TF.

### 4) MATLAB `tf2ss`
MATLAB’s realization is compared against the canonical realization; both are equivalent (same input-output behavior).

## Verification
- Same poles/zeros (within numerical tolerance)
- Step responses overlap
- `minreal(tf(G11_fromMIMO) - tf(G11_fromCanon))` returns zero


Open MATLAB and run:
- `matlab/dc_motor_g11_ss_tf_canonical.m`
