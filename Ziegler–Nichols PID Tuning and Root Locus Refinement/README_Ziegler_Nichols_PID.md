# Ziegler–Nichols PID Tuning and Root Locus Refinement

This repository demonstrates a PID controller design workflow for a linear plant using:

1. Ziegler–Nichols tuning rules  
2. Simulink closed-loop simulations  
3. MATLAB Control System Designer / root locus refinement  
4. Extraction of updated PID gains from a pseudo-derivative controller transfer function  

The project is based on classical control theory concepts and follows the PID tuning workflow demonstrated in Christopher Lum's AE511 Classical Control Theory material from the University of Washington.

---

## Plant Model

The plant transfer function used in this project is

```matlab
G_num = [1 -2 10];
G_den = [1 4 9 10 0];
Gs = tf(G_num,G_den);
```

which corresponds to

```math
G(s) = \frac{s^2 - 2s + 10}{s^4 + 4s^3 + 9s^2 + 10s}
```

The denominator has an integrator because of the pole at the origin.

---

## Ziegler–Nichols Parameters

The ultimate gain and ultimate period were obtained from neutral stability analysis using the Routh array / Simulink simulation:

```matlab
KU = 1.527;
TU = 4.767;
```

where:

- `KU` is the ultimate gain that makes the closed-loop system marginally stable.
- `TU` is the oscillation period at that gain.

These values are then used to compute PID gains for different Ziegler–Nichols tuning rules.

---

## Ziegler–Nichols Methods Compared

The following tuning rules are simulated and compared:

```matlab
TYPE = 'ClassicPID';
TYPE = 'PI';
TYPE = 'PessenIntegrationRule';
TYPE = 'SomeOvershoot';
TYPE = 'NoOvershoot';
```

Each method computes a different set of controller gains:

```matlab
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
```

The closed-loop response is then simulated in Simulink and plotted for comparison.

---

## Simulink Controller Structure

The PID controller is implemented as:

```math
u(t) = K_P e(t) + K_I \int e(t)\,dt + K_D \dot{e}(t)
```

However, instead of using a pure derivative, a pseudo-derivative filter is used:

```math
\frac{a s}{s+a}
```

This avoids the noise amplification and discontinuity problems caused by an ideal derivative.

The implemented controller has the form:

```math
C(s) = K_P + \frac{K_I}{s} + K_D \frac{a s}{s+a}
```

The equivalent transfer function is:

```math
C(s) =
\frac{(aK_D + K_P)s^2 + (aK_P + K_I)s + aK_I}
{s^2 + as}
```

In MATLAB:

```matlab
a = 100;

C_num = [a*KD+KP  a*KP+KI  a*KI];
C_den = [1         a        0];

Cs = tf(C_num,C_den);
```

---

## Initial Ziegler–Nichols Results

The initial Ziegler–Nichols comparison shows that the tuning rules have very different transient responses.

General observations:

| Method | Expected behavior |
|---|---|
| Classic PID | Fast response, noticeable overshoot |
| PI | Slower response, no derivative action |
| Pessen Integration Rule | Aggressive response, larger overshoot |
| Some Overshoot | Moderate tuning with controlled oscillation |
| No Overshoot | Slower and more conservative response |

The initial Ziegler–Nichols results are useful as a starting point, but not necessarily as the final controller.

---

## Root Locus Refinement

After obtaining an initial controller, the design was refined manually in MATLAB Control System Designer using root locus.

```matlab
controlSystemDesigner(Gs,Cs)
```

The compensator was adjusted interactively to improve the transient response.

The updated compensator was saved from Control System Designer as:

```matlab
mycontroller.mat
```

or as part of the full design session:

```matlab
mycontrolsystemdesign.mat
```

---

## Extracting the New PID Gains

After tuning the compensator in Control System Designer, the updated controller transfer function can be converted back into PID gains using:

```matlab
[KP_new, KI_new, KD_new, a_new] = ExtractPIDGainsFromSecondOrderTransferFunction(SYS);
```

This function assumes that the controller has the pseudo-derivative PID form:

```math
C(s) = K_P + \frac{K_I}{s} + K_D \frac{a s}{s+a}
```

and extracts the equivalent values of:

- `KP_new`
- `KI_new`
- `KD_new`
- `a_new`

These new gains can then be used again in the Simulink PID model to validate the improved closed-loop response.

---

## Suggested MATLAB Workflow

```matlab
clc
clear
close all

% Define plant
G_num = [1 -2 10];
G_den = [1 4 9 10 0];
Gs = tf(G_num,G_den);

% Ziegler-Nichols parameters
KU = 1.527;
TU = 4.767;

% Initial Ziegler-Nichols tuning
TYPE = 'ClassicPID';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);

% Pseudo-derivative PID controller
a = 100;
C_num = [a*KD+KP  a*KP+KI  a*KI];
C_den = [1         a        0];
Cs = tf(C_num,C_den);

% Open Control System Designer
controlSystemDesigner(Gs,Cs)

% After exporting the tuned compensator as SYS:
[KP_new, KI_new, KD_new, a_new] = ExtractPIDGainsFromSecondOrderTransferFunction(SYS);
```

---

## Files

Suggested repository structure:

```text
.
├── README.md
├── main_compare_ziegler_nichols.m
├── main_root_locus_refinement.m
├── ZieglerNichols.m
├── ExtractPIDGainsFromSecondOrderTransferFunction.m
├── ziegler_nichols_PIpseudoD.slx
├── mycontroller.mat
├── mycontrolsystemdesign.mat
└── figures
    ├── ziegler_nichols_comparison.png
    ├── refined_response_1.png
    ├── refined_response_2.png
    └── simulink_pid_structure.png
```

---

## Notes

- Ziegler–Nichols tuning is a practical starting point, not always the final design.
- The Pessen rule is usually more aggressive and may produce larger overshoot.
- The no-overshoot rule is more conservative but can be slower.
- Root locus refinement is useful for improving damping, overshoot, and settling time.
- The pseudo-derivative filter is important because a pure derivative is not physically realistic and amplifies high-frequency noise.

---

## References

This project uses helper functions and follows the classical control workflow presented by:

**Christopher Lum**  
Department of Aeronautics and Astronautics  
University of Washington  

Relevant material:

- AE511 Classical Control Theory
- Ziegler–Nichols PID tuning
- PID design using root locus
- MATLAB Control System Designer examples

The helper functions `ZieglerNichols.m` and `ExtractPIDGainsFromSecondOrderTransferFunction.m` are based on Christopher Lum's publicly shared educational MATLAB material. Please retain the original function headers and author attribution when using or modifying these files.

---

## Author

Prepared as part of a classical control theory study project involving PID tuning, root locus design, and Simulink validation.
