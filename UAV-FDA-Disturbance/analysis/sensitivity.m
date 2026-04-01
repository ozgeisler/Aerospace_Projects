% =========================================================
% sensitivity.m — S(jω), T(jω), PS(jω) Analysis
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

clear; clc;
load('linear_model.mat');

fprintf('=== Sensitivity Function Analysis ===\n');

w = logspace(-2, 2, 2000);

channels  = {'Roll','Pitch','Yaw','Heave'};
plant_sys = {sys_roll, sys_pitch, sys_yaw, sys_heave};
ctrl_sys  = {C_roll, C_pitch, C_yaw, C_heave};
colors_S  = {'#185FA5','#0F6E56','#993C1D','#7F77DD'};

%% ============================================================
%  Compute S, T, PS for each channel
% ============================================================
S_all  = cell(4,1);
T_all  = cell(4,1);
PS_all = cell(4,1);

hinf_S  = zeros(4,1);
hinf_T  = zeros(4,1);
w_peak  = zeros(4,1);
bw_S    = zeros(4,1);

for i = 1:4
    G = plant_sys{i};
    Ci = ctrl_sys{i};
    L  = Ci * G;

    S_all{i}  = feedback(1, L);           % S  = 1/(1+L)
    T_all{i}  = feedback(L, 1);           % T  = L/(1+L)
    PS_all{i} = feedback(G, Ci);          % PS = G/(1+L)

    % H-infinity norm of S
    [pk, wpk]   = norm(S_all{i}, inf);
    hinf_S(i)   = 20*log10(pk);
    w_peak(i)   = wpk;

    % H-infinity norm of T
    [pkt, ~]    = norm(T_all{i}, inf);
    hinf_T(i)   = 20*log10(pkt);

    % Bandwidth: frequency where |S(jω)| crosses -3 dB
    [mag_s, ~, wout] = bode(S_all{i}, w);
    mag_s_db = 20*log10(squeeze(mag_s));
    idx_bw = find(mag_s_db >= -3, 1, 'last');
    if ~isempty(idx_bw)
        bw_S(i) = wout(idx_bw);
    end
end

%% ============================================================
%  Summary Table
% ============================================================
fprintf('\n--- Sensitivity Metrics ---\n');
fprintf('%-8s  %10s  %10s  %12s  %10s\n', ...
    'Channel','||S||_inf','Peak @','||T||_inf','BW [r/s]');
fprintf('%s\n', repmat('-',1,58));
for i = 1:4
    fprintf('%-8s  %8.2f dB  %8.4f r/s  %8.2f dB  %8.4f\n', ...
        channels{i}, hinf_S(i), w_peak(i), hinf_T(i), bw_S(i));
end

%% ============================================================
%  1. Individual channel S, T, PS plots
% ============================================================
figure('Name','Sensitivity Functions','Position',[50 50 1400 900]);
sgtitle('Sensitivity Functions — All Channels', 'FontSize',14);

for i = 1:4
    [mag_s, ~, wout]  = bode(S_all{i},  w);
    [mag_t, ~]        = bode(T_all{i},  w);
    [mag_ps, ~]       = bode(PS_all{i}, w);

    ms_db  = 20*log10(squeeze(mag_s));
    mt_db  = 20*log10(squeeze(mag_t));
    mps_db = 20*log10(squeeze(mag_ps));

    subplot(2,2,i);
    semilogx(wout, ms_db,  'Color','#185FA5', 'LineWidth',1.8); hold on;
    semilogx(wout, mt_db,  'Color','#0F6E56', 'LineWidth',1.8, 'LineStyle','--');
    semilogx(wout, mps_db, 'Color','#993C1D', 'LineWidth',1.4, 'LineStyle',':');

    % Annotations
    yline(0,  'k:', '0 dB',  'LabelHorizontalAlignment','right', 'Alpha',0.5);
    yline(6,  'r--','6 dB limit','LabelHorizontalAlignment','right');
    xline(w_peak(i), '--', sprintf('\\omega_{peak}=%.2f', w_peak(i)), ...
        'Color','#185FA5','Alpha',0.6);

    grid on; xlabel('\omega [rad/s]'); ylabel('Magnitude [dB]');
    title(sprintf('%s Channel', channels{i}));
    legend('S(j\omega)','T(j\omega)','PS(j\omega)', 'Location','best');
    xlim([w(1) w(end)]);
end

saveas(gcf, fullfile('..','results','figures','sensitivity_all.png'));

%% ============================================================
%  2. Comparison: |S| across channels
% ============================================================
figure('Name','S Comparison','Position',[50 50 900 500]);

for i = 1:4
    [mag_s, ~, wout] = bode(S_all{i}, w);
    ms_db = 20*log10(squeeze(mag_s));
    semilogx(wout, ms_db, 'Color', colors_S{i}, ...
        'LineWidth', 1.8, 'DisplayName', channels{i}); hold on;
end

yline(0, 'k:', '0 dB', 'Alpha', 0.5);
yline(6, 'r--', '||S||_\infty = 6 dB limit');
grid on;
xlabel('\omega [rad/s]'); ylabel('|S(j\omega)| [dB]');
title('Sensitivity S(j\omega) — Channel Comparison');
legend('Location','southeast');
saveas(gcf, fullfile('..','results','figures','sensitivity_comparison.png'));

%% ============================================================
%  3. Waterbed effect illustration (Roll channel)
% ============================================================
figure('Name','Waterbed Effect','Position',[50 50 800 450]);

[mag_s, ~, wout] = bode(S_all{1}, w);
ms_db = 20*log10(squeeze(mag_s));

area_pos = ms_db;  area_pos(area_pos < 0) = 0;
area_neg = ms_db;  area_neg(area_neg > 0) = 0;

semilogx(wout, ms_db, 'Color','#185FA5','LineWidth',2); hold on;
fill([wout; flipud(wout)], [area_pos; zeros(size(area_pos))], ...
    '#185FA5', 'FaceAlpha', 0.15, 'EdgeColor','none');
fill([wout; flipud(wout)], [area_neg; zeros(size(area_neg))], ...
    '#993C1D', 'FaceAlpha', 0.15, 'EdgeColor','none');

yline(0,'k--','','LineWidth',0.8);
grid on;
xlabel('\omega [rad/s]'); ylabel('|S(j\omega)| [dB]');
title('Waterbed Effect — Roll Channel');
text(0.015, 4, 'Amplification', 'Color','#185FA5', 'FontSize',9);
text(0.015,-6, 'Attenuation',   'Color','#993C1D', 'FontSize',9);
annotation('textbox',[.15 .12 .4 .08],'String', ...
    '\int_{0}^{\infty} log|S(j\omega)| d\omega = 0  (Bode Integral)', ...
    'EdgeColor','none','FontSize',9,'Color',[0.4 0.4 0.4]);

saveas(gcf, fullfile('..','results','figures','waterbed.png'));

%% --- Save ---
save(fullfile('..','results','data','sensitivity_results.mat'), ...
    'S_all','T_all','PS_all','hinf_S','hinf_T','w_peak','bw_S','channels');

fprintf('\nSensitivity analysis complete. Figures saved.\n');
