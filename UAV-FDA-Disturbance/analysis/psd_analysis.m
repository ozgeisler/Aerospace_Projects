% =========================================================
% psd_analysis.m — Power Spectral Density / Welch Analysis
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

clear; clc;
load('linear_model.mat');

fprintf('=== PSD / Welch Disturbance Analysis ===\n');

fs   = 1 / p.dt;        % Sampling frequency [Hz]
t    = 0:p.dt:p.t_end;  % Time vector
N    = length(t);

rng(42);  % Reproducibility

%% ============================================================
%  1. Generate Dryden Wind Turbulence Signals
% ============================================================
% Dryden PSD (longitudinal): S_u(omega) = sigma^2 * L / (pi*V) * 1/(1+(L*w/V)^2)
% Approximate via white noise through Dryden shaping filter

fprintf('Generating Dryden turbulence signals...\n');

% White noise inputs
wn_u = randn(1, N);
wn_v = randn(1, N);
wn_w = randn(1, N);

% Dryden shaping filters (continuous, discretized)
% H_u(s) = sigma_u * sqrt(2*L_u/(pi*V)) / (1 + s*L_u/V)
tau_u = p.L_u / p.V_air;
tau_v = p.L_v / p.V_air;
tau_w = p.L_w / p.V_air;

H_u_c = tf(p.sigma_u * sqrt(2*tau_u/pi), [tau_u 1]);
H_v_c = tf(p.sigma_v * sqrt(2*tau_v/pi), [tau_v 1]);
H_w_c = tf(p.sigma_w * sqrt(2*tau_w/pi), [tau_w 1]);

H_u = c2d(H_u_c, p.dt, 'tustin');
H_v = c2d(H_v_c, p.dt, 'tustin');
H_w = c2d(H_w_c, p.dt, 'tustin');

% Filter white noise
d_u = lsim(H_u, wn_u, t);
d_v = lsim(H_v, wn_v, t);
d_w = lsim(H_w, wn_w, t);

fprintf('  sigma_u actual: %.3f m/s (target: %.3f)\n', std(d_u), p.sigma_u);
fprintf('  sigma_v actual: %.3f m/s (target: %.3f)\n', std(d_v), p.sigma_v);
fprintf('  sigma_w actual: %.3f m/s (target: %.3f)\n', std(d_w), p.sigma_w);

%% ============================================================
%  2. Simulate Disturbance Response (Roll & Heave channels)
% ============================================================
fprintf('Simulating disturbance responses...\n');

% Roll disturbance (via moment arm approximation: tau_d = m*g*l*phi_d)
% Use heave disturbance -> z output via closed-loop PS
d_heave_force = p.m * d_w;   % Force disturbance in Z [N]

% Response via PS transfer function
PS_heave_d = c2d(PS_all{4}, p.dt, 'tustin');
y_heave    = lsim(PS_heave_d, d_heave_force, t);

PS_roll_d  = c2d(PS_all{1}, p.dt, 'tustin');
d_roll_torque = 0.05 * d_u;   % Small roll disturbance torque
y_roll        = lsim(PS_roll_d, d_roll_torque, t);

%% ============================================================
%  3. Welch PSD Estimation
% ============================================================
nfft    = 2048;
noverlap = nfft / 2;
win      = hann(nfft);

% Disturbance PSD
[Puu, f_u] = pwelch(d_u,          win, noverlap, nfft, fs);
[Pvv, f_v] = pwelch(d_v,          win, noverlap, nfft, fs);
[Pww, f_w] = pwelch(d_w,          win, noverlap, nfft, fs);

% Response PSD
[Pyy_h, f_yh] = pwelch(y_heave,   win, noverlap, nfft, fs);
[Pyy_r, f_yr] = pwelch(y_roll,    win, noverlap, nfft, fs);

% Convert to rad/s
w_u  = 2*pi*f_u;
w_yh = 2*pi*f_yh;
w_yr = 2*pi*f_yr;

%% ============================================================
%  4. Theoretical Dryden PSD (for validation)
% ============================================================
w_th = logspace(-2, 2, 1000);
PSD_u_theory = (p.sigma_u^2 * 2*p.L_u/p.V_air) ./ (pi * (1 + (p.L_u*w_th/p.V_air).^2));
PSD_w_theory = (p.sigma_w^2 * p.L_w/p.V_air) ./ (pi * (1 + (p.L_w*w_th/p.V_air).^2));

%% ============================================================
%  5. Plot: Disturbance PSD
% ============================================================
figure('Name','Disturbance PSD','Position',[50 50 1200 500]);
sgtitle('Dryden Wind Turbulence — Power Spectral Density');

subplot(1,3,1);
loglog(w_u, Puu, 'Color','#185FA5','LineWidth',1.5); hold on;
loglog(w_th, PSD_u_theory, 'r--', 'LineWidth',1.2);
grid on; xlabel('\omega [rad/s]'); ylabel('PSD [(m/s)^2/(rad/s)]');
title('Longitudinal (u)'); legend('Welch','Dryden theory');

subplot(1,3,2);
loglog(2*pi*f_v, Pvv, 'Color','#0F6E56','LineWidth',1.5);
grid on; xlabel('\omega [rad/s]'); ylabel('PSD [(m/s)^2/(rad/s)]');
title('Lateral (v)');

subplot(1,3,3);
loglog(w_u, Pww, 'Color','#993C1D','LineWidth',1.5); hold on;
loglog(w_th, PSD_w_theory,'r--','LineWidth',1.2);
grid on; xlabel('\omega [rad/s]'); ylabel('PSD [(m/s)^2/(rad/s)]');
title('Vertical (w)'); legend('Welch','Dryden theory');

saveas(gcf, fullfile('..','results','figures','psd_disturbance.png'));

%% ============================================================
%  6. Plot: Response PSD
% ============================================================
figure('Name','Response PSD','Position',[50 50 1000 450]);
sgtitle('Closed-Loop Disturbance Response PSD');

subplot(1,2,1);
semilogy(w_yh, Pyy_h, 'Color','#185FA5','LineWidth',1.5);
grid on; xlabel('\omega [rad/s]'); ylabel('PSD [m^2/(rad/s)]');
title('Heave response to w-disturbance');

subplot(1,2,2);
semilogy(w_yr, Pyy_r, 'Color','#0F6E56','LineWidth',1.5);
grid on; xlabel('\omega [rad/s]'); ylabel('PSD [rad^2/(rad/s)]');
title('Roll response to u-disturbance');

saveas(gcf, fullfile('..','results','figures','psd_response.png'));

%% ============================================================
%  7. Dominant Frequency Extraction
% ============================================================
fprintf('\n--- Dominant Disturbance Frequencies ---\n');
[~, idx_u]  = max(Puu); fprintf('  Longitudinal peak: %.4f rad/s\n', w_u(idx_u));
[~, idx_w]  = max(Pww); fprintf('  Vertical peak:     %.4f rad/s\n', w_u(idx_w));
[~, idx_yh] = max(Pyy_h); fprintf('  Heave resp peak:  %.4f rad/s\n', w_yh(idx_yh));
[~, idx_yr] = max(Pyy_r); fprintf('  Roll resp peak:   %.4f rad/s\n', w_yr(idx_yr));

% Total power (integrate PSD)
df_u  = mean(diff(f_u));
P_tot_u = sum(Puu) * df_u;
P_tot_w = sum(Pww) * df_u;
fprintf('\n--- Total Disturbance Power ---\n');
fprintf('  Longitudinal: %.4f (m/s)^2\n', P_tot_u);
fprintf('  Vertical:     %.4f (m/s)^2\n', P_tot_w);

%% --- Save ---
save(fullfile('..','results','data','psd_results.mat'), ...
    'd_u','d_v','d_w','y_heave','y_roll', ...
    'Puu','Pvv','Pww','Pyy_h','Pyy_r', ...
    'f_u','f_v','f_w','f_yh','f_yr', 't');

fprintf('\nPSD analysis complete. Figures saved.\n');
