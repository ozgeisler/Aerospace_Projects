# Planar Vehicle Dynamics Simulation

A nonlinear planar vehicle dynamics simulation developed in MATLAB/Simulink for studying equations of motion, trimming, linearization, and classical control analysis.

This project was developed as part of my self-study in flight dynamics and control. Although the system represents a simplified planar vehicle, the workflow closely follows that used for aircraft flight dynamics: nonlinear modeling, trim analysis, operating-point linearization, and transfer function extraction.

---

## Features

- Nonlinear 3-DOF planar vehicle model
- MATLAB/Simulink implementation
- Modular vehicle parameter definition
- Time-domain simulation
- Trim point computation using Simulink Linear Analysis Tool
- Operating point linearization
- State-space model extraction
- Transfer function generation
- Ready for classical and modern control design

---

## Vehicle States

The nonlinear model consists of six states

\[
x=
\begin{bmatrix}
x\\
y\\
\dot{x}\\
\dot{y}\\
\theta\\
\dot{\theta}
\end{bmatrix}
\]

where

| State | Description |
|--------|-------------|
| x | Global x-position |
| y | Global y-position |
| xDot | Velocity in x-direction |
| yDot | Velocity in y-direction |
| θ | Heading angle |
| θDot | Angular velocity |

---

## Control Inputs

The vehicle is controlled using two normalized inputs

\[
u=
\begin{bmatrix}
T_f\\
A_f
\end{bmatrix}
\]

where

| Input | Description |
|--------|-------------|
| Thrust Fraction | Longitudinal propulsion command |
| Axial Fraction | Steering / differential thrust command |

---

## Dynamic Model

The nonlinear equations of motion include

- Translational dynamics
- Rotational dynamics
- Propulsion model
- Aerodynamic drag
- Rotational damping

The simulation is implemented entirely in Simulink using modular subsystems.

---

## Project Structure

```
Planar-Vehicle-Dynamics-Simulation
│
├── EOM_planar_vehicle.slx
├── EOM_planar_vehicle_linmod.slx
├── run_eom.m
├── LineariseScript.m
├── PlanarVehicleLoadConstants.m
├── TrimPoint01.mat
├── LinearSystem01.mat
├── Boat.png
├── Figure1.png
├── Figure2.png
└── README.md
```

---

## Running the Simulation

Load the parameters

```matlab
clear
clc
close all

deltaTMax = 0.05;

m  = 10;
Ib = 0.5;

Fmax = 10;
Mmax = 0.5;

CT = 1/40;
CR = 0.75;

x0 = [10;
     -10;
      0;
     15;
     pi/2;
      0];

sim('EOM_planar_vehicle.slx')
```

---

## Trim Analysis

The nonlinear model was trimmed using the Simulink Linear Analysis Tool.

The selected operating condition was

```matlab
x0 = [10;
     -10;
      0;
     10;
     pi/2;
      0];

u0 = [0.25;
      0];
```

The trimming process computes an equilibrium operating point that satisfies the desired steady-state constraints.

Once the trim solution is obtained, the operating point is exported and used for linearization.

---

## Linearization

After trimming, the nonlinear model is linearized around the operating point.

The resulting state-space model is

\[
\dot{x}=Ax+Bu
\]

\[
y=Cx+Du
\]

The matrices are extracted directly from the Linear Analysis Tool

```matlab
A = linsys1.A;
B = linsys1.B;
C = linsys1.C;
D = linsys1.D;
```

---

## Transfer Function Extraction

The state-space model is converted into transfer functions using

```matlab
[num_11,den_11] = ss2tf(A,B(:,1),C(1,:),D(1,1));
[num_12,den_12] = ss2tf(A,B(:,2),C(1,:),D(1,2));

[num_21,den_21] = ss2tf(A,B(:,1),C(2,:),D(2,1));
[num_22,den_22] = ss2tf(A,B(:,2),C(2,:),D(2,2));

G11 = minreal(tf(num_11,den_11));
G12 = minreal(tf(num_12,den_12));

G21 = minreal(tf(num_21,den_21));
G22 = minreal(tf(num_22,den_22));
```

This enables

- Pole-zero analysis
- Frequency response analysis
- Classical controller design
- Stability analysis

---

## Alternative Linearization

The project also demonstrates linearization using MATLAB's `linmod`

```matlab
[A,B,C,D] = linmod('EOM_planar_vehicle_linmod',x0,u0);

[num,den] = ss2tf(A,B,C,D);

G = minreal(tf(num,den));
```

This allows comparison between different linearization workflows.

---

## Results

The project demonstrates the complete workflow

✔ Nonlinear dynamic simulation

✔ Trim point computation

✔ Operating-point linearization

✔ State-space model extraction

✔ Transfer function generation

✔ Classical control model preparation

---


## Learning Objectives

This project was developed to gain practical experience with

- Nonlinear dynamic modeling
- Flight dynamics workflows
- Simulink modeling
- Numerical trimming
- Linearization techniques
- State-space analysis
- Transfer function derivation
- Classical control theory

The workflow closely mirrors the procedures used in aircraft flight dynamics before designing flight controllers.

---

