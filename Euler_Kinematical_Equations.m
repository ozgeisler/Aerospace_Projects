clc
clear all
close

%Load gyro-dat
load('GyroData.mat')

%Define Initial Conditions for Euler Angles
phi0   = 0;
theta0 = 0.0059;
psi0   = 0;


%Define initial DCM for Poisson's Kinematical Equations
C1v = [cos(psi0) sin(psi0) 0;
    -sin(psi0) cos(psi0) 0;
    0 0 1];
C21 = [cos(theta0) 0 -sin(theta0);
    0 1 0;
    sin(theta0) 0 cos(theta0)];
Cb2 = [1 0 0;
    0 cos(phi0) sin(phi0);
    0 -sin(phi0) cos(phi0)];

Cbv0 = Cb2*C21*C1v;

%Define initial quaternions

q0 = angle2quat(psi0,theta0,phi0)

open ('Model_Initialize.slx')

sim ('Model_Initialize.slx')