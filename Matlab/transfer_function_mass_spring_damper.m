clc 
clear
close all

%Define the constants of the transfer funtions
m = 1;
c = 3;
k = 5;

%Create a transfer function object in Matlab (using the 'tf' command)
%
% G (s) = 1/(m*s^2+c*s+k)
G_num = [1];
G_den = [m c k];

G = tf(G_num,G_den)

%Subject the system to a unit step input
figure
tFinal = 10
step(G,tFinal)

%Subject the system to a sinusoidal input(u(t)= 5*sin(2*t)