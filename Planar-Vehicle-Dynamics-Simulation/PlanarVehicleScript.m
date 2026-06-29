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
x0 = [10; -10; 0; 15; pi/2; 0];
sim('EOM_planar_vehicle.slx')
open('EOM_planar_vehicle.slx')