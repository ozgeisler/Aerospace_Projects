clc 
clear
close all

% Defining the transfer function throtle w/thrust force relations

thrust_force = [500000];
throttle = [1 2];

GE_s = tf(thrust_force,throttle)

%Subject the system to a unit step input
figure
title('Thrust-Throttle Transfer Function')
tFinal = 1;
step(GE_s,tFinal)
title('Thrust Force Response to Step Force')

% Defining transfer function of the catapult force and the given catapult force command
catapultforce = [5200000];
catapultcommand = [1 4 13];
GC_s = tf(catapultforce,catapultcommand)


%Subject the system to a unit step input
figure
step(GC_s,tFinal)
title('Catapul Response to Step Force')
%Defining the constants of F-14 properties 
W = 230000;
g = 9.81;
m = W/g
c = 100;
density = 1.225;
lift_coefficient = 1.4;
wing_area = 54.5;

%Defining the lift equation's constant value
K = 1/2*(density)*(lift_coefficient)*(wing_area) 

%Defining the position transfer function F-14 aircraft
Gp_num = [1];
Gp_den = [m c 0];
Gp_s = tf(Gp_num,Gp_den)

%Definig the velocity transfer function F-14 aircraft
Gv_num = [1];
Gv_den = [m c];
Gv_s = tf(Gv_num,Gv_den)

open('F14_carrier_launch.slx')
sim("F14_carrier_launch.slx")
