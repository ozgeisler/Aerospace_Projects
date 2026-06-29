%Demonstrate how to trim a model using the Linear Analysis  Tool
clear
clc
close all

deltaTMax = 0.05;


%% Define model parameters
m = 10;
Ib = 0.5;

%Propulsion
Fmax = 10;
Mmax = 0.5;

%Environment
CT = 1/40;
CR = 0.75;

%Simulation
x0 = [10; -10; 0; 10; pi/2; 0];
u0 = [0.25;
    0];
% Obtain the transfer function for the system
A = linsys1.A;
B = linsys1.B;
C = linsys1.C;
D = linsys1.D;

[num_11,den_11] = ss2tf(A,B(:,1),C(1,:),D(1,1));
[num_12,den_12] = ss2tf(A,B(:,2),C(1,:),D(1,2));
[num_21,den_21] = ss2tf(A,B(:,1),C(2,:),D(2,1));
[num_22,den_22] = ss2tf(A,B(:,2),C(2,:),D(2,2));

G_11 = minreal(tf(num_11,den_11))
G_12 = minreal(tf(num_12,den_12))
G_21 = minreal(tf(num_21,den_21))
G_22 = minreal(tf(num_22,den_22))

% using 'linmod' to linearize

% [A,B,C,D] = linmod('EOM_planar_vehicle_linmod',x0,u0(1));
% 
% [num,den] = ss2tf(A,B,C,D);
% 
% GV = minreal(tf(num,den))




