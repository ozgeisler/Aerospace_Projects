% =========================================================
% main_run.m — Master Script: Run Full FDA Pipeline
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================
% Run this file to execute the complete analysis pipeline.
% Requires: MATLAB R2022a+, Control System Toolbox,
%           Signal Processing Toolbox, Simulink (for .slx)
% =========================================================

clear; clc; close all;
fprintf('============================================================\n');
fprintf('  UAV Frequency Domain Analysis — Disturbance Sensitivity\n');
fprintf('  Generic 6-DOF Model | Hover Trim | MATLAB/Simulink\n');
fprintf('============================================================\n\n');

%% --- Setup paths ---
root   = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'models'));
addpath(fullfile(root,'analysis'));
addpath(fullfile(root,'utils'));

cd(fullfile(root,'models'));

%% === STEP 1: Load Parameters ===
fprintf('[1/4] Loading UAV parameters...\n');
run('params.m');
fprintf('      Done.\n\n');

%% === STEP 2: Linearize Model ===
fprintf('[2/4] Linearizing 6-DOF model at hover trim...\n');
run('linear_model.m');
fprintf('      Done.\n\n');

%% === STEP 3: Frequency Domain Analysis ===
cd(fullfile(root,'analysis'));

fprintf('[3/4] Running frequency domain analysis...\n');

fprintf('  [3a] Bode & Nyquist analysis...\n');
run('bode_analysis.m');

fprintf('  [3b] Sensitivity function analysis...\n');
run('sensitivity.m');

fprintf('  [3c] PSD / Welch analysis...\n');
run('psd_analysis.m');

fprintf('      Done.\n\n');

%% === STEP 4: Export Report ===
fprintf('[4/4] Exporting HTML report...\n');
cd(fullfile(root,'utils'));
export_report(fullfile(root,'results','UAV_FDA_Report.html'));
fprintf('      Done.\n\n');

%% === Summary ===
fprintf('============================================================\n');
fprintf('  ANALYSIS COMPLETE\n');
fprintf('============================================================\n');
fprintf('  Results:  %s\n', fullfile(root,'results'));
fprintf('  Report:   %s\n', fullfile(root,'results','UAV_FDA_Report.html'));
fprintf('  Figures:  %s\n', fullfile(root,'results','figures'));
fprintf('============================================================\n');

%% --- Quick stability check ---
load(fullfile(root,'results','data','bode_results.mat'));
load(fullfile(root,'results','data','sensitivity_results.mat'));

channels = {'Roll','Pitch','Yaw','Heave'};
fprintf('\n  Quick Summary:\n');
fprintf('  %-8s  %-10s  %-10s  %-12s\n','Channel','GM [dB]','PM [°]','||S||_inf');
fprintf('  %s\n', repmat('-',1,48));
for i = 1:4
    gm_ok = margin_table(i,1) >= 6;
    pm_ok = margin_table(i,2) >= 30;
    s_ok  = hinf_S(i) <= 6;
    ok_str = '';
    if gm_ok && pm_ok && s_ok, ok_str = '✓ OK';
    elseif ~gm_ok || ~pm_ok,   ok_str = '⚠ CHECK';
    else,                       ok_str = '✗ FAIL'; end
    fprintf('  %-8s  %-10.2f  %-10.2f  %-12.2f  %s\n', ...
        channels{i}, margin_table(i,1), margin_table(i,2), hinf_S(i), ok_str);
end
fprintf('\n');
