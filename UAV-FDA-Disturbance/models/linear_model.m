% =========================================================
% linear_model.m — 6-DOF UAV Linearization at Hover Trim
% UAV Frequency Domain Analysis — Disturbance Sensitivity
% =========================================================

clear; clc;
run('params.m');

fprintf('=== Linearization at Hover Trim Point ===\n');

%% --- Trim State (hover) ---
% State vector: x = [u v w  p q r  phi theta psi  x y z]
%               (body vel, ang vel, Euler angles, position)
% Input vector: u = [T tau_phi tau_theta tau_psi]
%               (total thrust, roll/pitch/yaw torques)

x_trim = zeros(12, 1);  % All states zero at hover
u_trim = [p.m * p.g; 0; 0; 0];  % Thrust = weight, zero torques

%% --- Analytical A, B matrices (hover linearization) ---

% Translational dynamics (body frame)
A = zeros(12, 12);

% u_dot = -g*theta,  v_dot = g*phi  (gravity coupling)
A(1, 8)  = -p.g;   % du/dtheta
A(2, 7)  =  p.g;   % dv/dphi

% Angular dynamics: p_dot, q_dot, r_dot
A(4, 4)  = 0;
A(5, 5)  = 0;
A(6, 6)  = 0;

% Kinematic: phi_dot = p, theta_dot = q, psi_dot = r (small angle)
A(7, 4)  = 1;
A(8, 5)  = 1;
A(9, 6)  = 1;

% Position: x_dot = u, y_dot = v, z_dot = w (inertial frame)
A(10, 1) = 1;
A(11, 2) = 1;
A(12, 3) = 1;

% Aerodynamic drag (linearized)
A(1, 1)  = -p.Ax / p.m;
A(2, 2)  = -p.Ay / p.m;
A(3, 3)  = -p.Az / p.m;

% Input matrix B
B = zeros(12, 4);
B(3, 1)  = -1 / p.m;           % Thrust -> w_dot
B(4, 2)  =  1 / p.Ixx;         % tau_phi   -> p_dot
B(5, 3)  =  1 / p.Iyy;         % tau_theta -> q_dot
B(6, 4)  =  1 / p.Izz;         % tau_psi   -> r_dot

% Output matrix C (full state output)
C = eye(12);
D = zeros(12, 4);

%% --- State Space Model ---
states  = {'u','v','w','p','q','r','phi','theta','psi','x','y','z'};
inputs  = {'T','tau_phi','tau_theta','tau_psi'};
outputs = states;

sys_ss = ss(A, B, C, D, ...
    'StateName',  states, ...
    'InputName',  inputs, ...
    'OutputName', outputs);

fprintf('System order: %d\n', order(sys_ss));
fprintf('Number of inputs:  %d\n', size(B,2));
fprintf('Number of outputs: %d\n', size(C,1));

%% --- Eigenvalue Analysis ---
ev = eig(A);
fprintf('\n--- Open-loop Eigenvalues ---\n');
for i = 1:length(ev)
    if imag(ev(i)) >= 0
        fprintf('  lambda_%02d = %+.4f %+.4fi\n', i, real(ev(i)), imag(ev(i)));
    end
end

n_unstable = sum(real(ev) > 0);
fprintf('\nUnstable poles: %d\n', n_unstable);

%% --- Decoupled Channel Transfer Functions ---
% Roll channel: tau_phi -> phi
sys_roll  = tf(sys_ss(7, 2));
% Pitch channel: tau_theta -> theta
sys_pitch = tf(sys_ss(8, 3));
% Yaw channel: tau_psi -> psi
sys_yaw   = tf(sys_ss(9, 4));
% Heave channel: T -> z
sys_heave = tf(sys_ss(12, 1));

fprintf('\n--- Channel Transfer Functions ---\n');
fprintf('Roll  (tau_phi -> phi):\n');   disp(sys_roll);
fprintf('Pitch (tau_theta -> theta):\n'); disp(sys_pitch);
fprintf('Yaw   (tau_psi -> psi):\n');    disp(sys_yaw);
fprintf('Heave (T -> z):\n');            disp(sys_heave);

%% --- PID Controller Design (baseline) ---
% Simple PD controller for roll/pitch
Kp_rp = 8; Kd_rp = 3; N_rp = 50;
C_roll  = pid(Kp_rp, 0, Kd_rp, 1/N_rp);
C_pitch = pid(Kp_rp, 0, Kd_rp, 1/N_rp);

Kp_y = 3; Kd_y = 1; N_y = 30;
C_yaw   = pid(Kp_y, 0, Kd_y, 1/N_y);

Kp_h = 5; Ki_h = 2;
C_heave = pid(Kp_h, Ki_h, 0);

%% --- Closed-loop Systems ---
CL_roll  = feedback(C_roll  * sys_roll,  1);
CL_pitch = feedback(C_pitch * sys_pitch, 1);
CL_yaw   = feedback(C_yaw   * sys_yaw,  1);
CL_heave = feedback(C_heave * sys_heave, 1);

fprintf('\n--- Closed-loop Stability ---\n');
channels = {'Roll','Pitch','Yaw','Heave'};
cl_sys   = {CL_roll, CL_pitch, CL_yaw, CL_heave};
for i = 1:4
    p_cl = pole(cl_sys{i});
    stable = all(real(p_cl) < 0);
    fprintf('  %s: %s\n', channels{i}, string(stable).replace('true','STABLE').replace('false','UNSTABLE'));
end

%% --- Save ---
save('linear_model.mat', 'sys_ss', 'A', 'B', 'C', 'D', ...
    'sys_roll', 'sys_pitch', 'sys_yaw', 'sys_heave', ...
    'C_roll', 'C_pitch', 'C_yaw', 'C_heave', ...
    'CL_roll', 'CL_pitch', 'CL_yaw', 'CL_heave', 'p');

fprintf('\nLinear model saved to linear_model.mat\n');
