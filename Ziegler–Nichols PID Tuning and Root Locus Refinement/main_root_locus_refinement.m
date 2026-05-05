clc
clear
close all

% Define the transfer function
G_num = [1 -2 10];
G_den = [1 4 9 10 0];
Gs = tf(G_num,G_den)


KU = 1.527; % for neutral stability extract from Routh array/Simulink
TU = 4.767; % period of Ku 
[KP, KI, KD] = ZieglerNichols(KU, TU, 'ClassicPID')

% Defining the PI Psedu-Derivative Controller,from ziegler Nichols data

a = 100;
C_num = [a*KD+KP a*KP+KI a*KI];
C_den = [1 a 0];
Cs = tf(C_num,C_den)

controlSystemDesigner(Gs,Cs)

% applying new compensator's transfer function and found new KP,KI,KD
[KP_new,KI_new,KD_new,a_new] = ExtractPIDGainsFromSecondOrderTransferFunction(SYS)

