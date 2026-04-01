% =========================================================
% bode_analysis.m — Bode / Nyquist Frequency Response
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

clear; clc;
load('linear_model.mat');

fprintf('=== Bode & Nyquist Analysis ===\n');

w = logspace(-2, 2, 1000);  % Frequency vector [rad/s]

channels     = {'Roll','Pitch','Yaw','Heave'};
plant_sys    = {sys_roll, sys_pitch, sys_yaw, sys_heave};
ctrl_sys     = {C_roll, C_pitch, C_yaw, C_heave};
colors       = {'#185FA5','#0F6E56','#993C1D','#7F77DD'};

%% ============================================================
%  1. Open-loop Bode Plots
% ============================================================
figure('Name','Open-Loop Bode','Position',[50 50 1200 800]);
sgtitle('Open-Loop Bode Diagrams — All Channels', 'FontSize', 14);

for i = 1:4
    L = ctrl_sys{i} * plant_sys{i};  % Loop TF

    [mag, phase, wout] = bode(L, w);
    mag_db = 20*log10(squeeze(mag));
    ph_deg = squeeze(phase);

    % Magnitude
    subplot(4, 2, 2*i-1);
    semilogx(wout, mag_db, 'Color', colors{i}, 'LineWidth', 1.5);
    hold on;
    yline(0, 'k--', '0 dB', 'LabelHorizontalAlignment','left');
    grid on; xlabel('Frequency [rad/s]'); ylabel('|L(j\omega)| [dB]');
    title(sprintf('%s — Magnitude', channels{i}));
    xlim([w(1) w(end)]);

    % Phase
    subplot(4, 2, 2*i);
    semilogx(wout, ph_deg, 'Color', colors{i}, 'LineWidth', 1.5);
    hold on;
    yline(-180, 'r--', '-180°', 'LabelHorizontalAlignment','left');
    grid on; xlabel('Frequency [rad/s]'); ylabel('\angle L(j\omega) [deg]');
    title(sprintf('%s — Phase', channels{i}));
    xlim([w(1) w(end)]);
end

saveas(gcf, fullfile('..','results','figures','bode_openloop.png'));

%% ============================================================
%  2. Stability Margins
% ============================================================
fprintf('\n--- Stability Margins ---\n');
fprintf('%-8s  %8s  %8s  %10s  %10s\n', ...
    'Channel','GM [dB]','PM [deg]','Wcg [r/s]','Wcp [r/s]');
fprintf('%s\n', repmat('-',1,55));

margin_table = zeros(4, 4);
for i = 1:4
    L = ctrl_sys{i} * plant_sys{i};
    [gm, pm, wcg, wcp] = margin(L);
    gm_db = 20*log10(gm);
    margin_table(i,:) = [gm_db, pm, wcg, wcp];
    fprintf('%-8s  %8.2f  %8.2f  %10.4f  %10.4f\n', ...
        channels{i}, gm_db, pm, wcg, wcp);
end

%% ============================================================
%  3. Gain & Phase Margin Plot (summary)
% ============================================================
figure('Name','Stability Margins','Position',[50 50 900 400]);

subplot(1,2,1);
bar(margin_table(:,1), 'FaceColor', '#185FA5', 'EdgeColor', 'none');
set(gca,'XTickLabel', channels);
yline(6, 'r--', '6 dB min'); grid on;
ylabel('Gain Margin [dB]'); title('Gain Margins');

subplot(1,2,2);
bar(margin_table(:,2), 'FaceColor', '#0F6E56', 'EdgeColor', 'none');
set(gca,'XTickLabel', channels);
yline(30, 'r--', '30° min'); grid on;
ylabel('Phase Margin [deg]'); title('Phase Margins');

sgtitle('Stability Margin Summary');
saveas(gcf, fullfile('..','results','figures','stability_margins.png'));

%% ============================================================
%  4. Nyquist Plots
% ============================================================
figure('Name','Nyquist Diagrams','Position',[50 50 1000 800]);
sgtitle('Nyquist Diagrams — All Channels');

for i = 1:4
    L = ctrl_sys{i} * plant_sys{i};
    subplot(2, 2, i);
    nyquist(L);
    title(channels{i}); grid on;
end

saveas(gcf, fullfile('..','results','figures','nyquist.png'));

%% --- Save results ---
save(fullfile('..','results','data','bode_results.mat'), ...
    'margin_table', 'channels', 'w');

fprintf('\nBode analysis complete. Figures saved.\n');
