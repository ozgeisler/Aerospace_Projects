clc 
clear 
close all

%Define individual transfer functions
G1_num = [3 -9];
G1_den = [1 7 12];

G2_num = [-2 -42 -142 -102];
G2_den = [1 26 197 408 720];

H_num = [3 6 15];
H_den = [1 13 56 92 48];

%Define the closed loop transfer function
T_num = [-2 -80 -1160 -7808 -25766 -40136 -25992 -4896];
T_den = [1 42 705 6176 31370 97888 195780 240192 142830];

simulinkModel = 'block_diagram';
open(simulinkModel)
sim(simulinkModel)



