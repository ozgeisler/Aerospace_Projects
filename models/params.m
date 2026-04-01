% =========================================================
% params.m — Generic 6-DOF UAV Physical Parameters
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

clear; clc;

%% --- Mass & Inertia ---
p.m   = 1.5;          % Total mass [kg]
p.g   = 9.81;         % Gravitational acceleration [m/s^2]

% Inertia tensor (diagonal, symmetric airframe assumed)
p.Ixx = 0.0347;       % Roll axis  [kg·m^2]
p.Iyy = 0.0458;       % Pitch axis [kg·m^2]
p.Izz = 0.0977;       % Yaw axis   [kg·m^2]
p.Ixy = 0; p.Ixz = 0; p.Iyz = 0;

%% --- Rotor / Actuator ---
p.n_rotors = 4;       % Number of rotors
p.l        = 0.225;   % Arm length [m]
p.kF       = 6.11e-8; % Thrust coefficient   [N/(rad/s)^2]
p.kM       = 1.50e-9; % Torque coefficient   [N·m/(rad/s)^2]
p.km       = 20;      % Motor gain [rad/s per unit input]
p.tau_m    = 0.02;    % Motor time constant [s]

% Hover rotor speed
p.omega_h  = sqrt(p.m * p.g / (p.n_rotors * p.kF));

%% --- Aerodynamic Drag ---
p.Ax = 0.25;   % Drag coefficient X [N·s/m]
p.Ay = 0.25;   % Drag coefficient Y [N·s/m]
p.Az = 0.30;   % Drag coefficient Z [N·s/m]

%% --- Disturbance Model (Dryden Wind) ---
p.sigma_u = 1.5;   % Turbulence intensity X [m/s]
p.sigma_v = 1.5;   % Turbulence intensity Y [m/s]
p.sigma_w = 0.75;  % Turbulence intensity Z [m/s]
p.L_u     = 200;   % Turbulence scale length X [m]
p.L_v     = 200;   % Turbulence scale length Y [m]
p.L_w     = 50;    % Turbulence scale length Z [m]
p.V_air   = 10;    % Nominal airspeed [m/s]

%% --- Simulation Settings ---
p.dt      = 0.001;   % Sample time [s]
p.t_end   = 30;      % Simulation duration [s]
p.freq_min = 0.01;   % Min frequency for FDA [rad/s]
p.freq_max = 100;    % Max frequency for FDA [rad/s]

%% --- Save ---
save('params.mat', 'p');
fprintf('UAV parameters loaded. Hover speed: %.1f rad/s\n', p.omega_h);
