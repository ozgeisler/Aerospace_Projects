# Attitude-Kinematics-Comparison-from-Gyroscope-Data
This project implements and compares three classical rigid body kinematic formulations for attitude propagation using real gyroscope measurements:

Euler Angle Kinematic Equations

Poisson’s Kinematic Equation (Direction Cosine Matrix propagation)

Quaternion-based Attitude Propagation

The goal is to evaluate numerical behavior, singularity characteristics, and consistency between representations when integrating angular velocity data.

The gyroscope measurements 
(
𝑝
,
𝑞
,
𝑟
)
(p,q,r) are loaded from GyroData.mat and propagated through three parallel Simulink implementations.
