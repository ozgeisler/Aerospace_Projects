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


tFinal = 15;
%Compute and sim for various Ziegler Nichols techniques

TYPE = 'ClassicPID';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
out = sim('ziegler_nichols.slx');
simY = out.get('simY');
t1 = simY.Time;
y1 = simY.Data;

TYPE = 'PI';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
out = sim('ziegler_nichols.slx');
simY = out.get('simY');
t2 = simY.Time;
y2 = simY.Data;

TYPE = 'PessenIntegrationRule';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
out = sim('ziegler_nichols.slx');
simY = out.get('simY');
t3 = simY.Time;
y3 = simY.Data;

TYPE = 'SomeOvershoot';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
out = sim('ziegler_nichols.slx');
simY = out.get('simY');
t4 = simY.Time;
y4 = simY.Data;

TYPE = 'NoOvershoot';
[KP, KI, KD] = ZieglerNichols(KU, TU, TYPE);
out = sim('ziegler_nichols.slx');
simY = out.get('simY');
t5 = simY.Time;
y5 = simY.Data;

figure
plot(t1,y1,...
    t2,y2,...
    t3,y3,...
    t4,y4,...
    t5,y5,...
    'LineWidth', 2)
grid on
title('Ziegler Nichols')
legend('Classic PID','PI','Pessen Integration Rule','Some Overshoot','No Overshoot')