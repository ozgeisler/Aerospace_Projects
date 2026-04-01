%% Compare G(s) vs G_tilde(s)
clc
clear 
close all

% Parameters
p3 = 50; % The third pole example
t = linspace(0, 10, 1000);

% 1.Original System G(s)
% G(s) = (s+2) / [ (s+1+3i)(s+1-3i) * (1/p3*s + 1) ]
num = [1 2];
den_p1p2 = [1 2 10]; % (s+1+3i)(s+1-3i) = s^2 + 2s + 10
den_p3 = [1/p3 1];
den_total = conv(den_p1p2, den_p3);
G = tf(num, den_total);

% 2.Approximation System G_tilde(s)
G_tilde = tf(num, den_p1p2);

% Step Responses
[y, t] = step(G, t);
[y_tilde, ~] = step(G_tilde, t);


% G(s) Analysis
po_g = CalculatePercentOvershoot(t, y);
ts_g = SettlingTime(t, y, 0.04); % %4 band
tr_g = RiseTime(t, y, 0.1, 0.8); % %10-%80

% Approximation G_tilde(s) Analysis
po_tilde = CalculatePercentOvershoot(t, y_tilde);
ts_tilde = SettlingTime(t, y_tilde, 0.04);
tr_tilde = RiseTime(t, y_tilde, 0.1, 0.8);

%% The Results
fprintf('--- G(s) Results ---\n');
fprintf('Overshoot: %.2f %%\n', po_g);
fprintf('Settling Time: %.2f s\n', ts_g);
fprintf('Rise Time: %.2f s\n\n', tr_g);

fprintf('--- G_tilde(s) Results ---\n');
fprintf('Overshoot: %.2f %%\n', po_tilde);
fprintf('Settling Time: %.2f s\n', ts_tilde);
fprintf('Rise Time: %.2f s\n', tr_tilde);

% Comparing the plots
plot(t, y, 'b', 'LineWidth', 1.5); 
hold on;
plot(t, y_tilde, 'r--', 'LineWidth', 1.5);
grid on; 
legend('Original G(s)', 'Approximation G_{tilde}(s)');
title('Step Response Comparisions');

p3_values = [0.1, 1, 3, 10];
t = linspace(0, 20, 1000);
figure;

for i = 1:length(p3_values)
    p3 = p3_values(i);
    % Definitions
    num = [1 2];
    den_g = conv([1 2 10], [1/p3 1]);
    G = tf(num, den_g);
    G_tilde = tf([1 2], [1 2 10]);
    
    % Step Response
    [y_g, ~] = step(G, t);
    [y_tilde, ~] = step(G_tilde, t);
    
    % The plots
    subplot(2, 2, i);
    plot(t, y_g, 'b', t, y_tilde, 'r--');
    title(['p_3 = ', num2str(p3)]);
    legend('G(s)', 'G_{tilde}(s)'); grid on;
end
p3_range = linspace(0.1, 10, 100);

% Time
t_sim = linspace(0, 30, 2000); 

% Approximation System Characteristics (G_tilde)
num_tilde = [1 2];
den_tilde = [1 2 10];
[y_tilde, t_tilde] = step(tf(num_tilde, den_tilde), t_sim);

po_tilde = CalculatePercentOvershoot(t_tilde, y_tilde);
ts_tilde = SettlingTime(t_tilde, y_tilde, 0.04);
tr_tilde = RiseTime(t_tilde, y_tilde, 0.1, 0.8);

% Loop for G(s)
po_vals = zeros(size(p3_range));
ts_vals = zeros(size(p3_range));
tr_vals = zeros(size(p3_range));

for j = 1:length(p3_range)
    p3 = p3_range(j);
    
    % G(s) denominator: (s^2 + 2s + 10) * (s/p3 + 1)
    den_full = conv(den_tilde, [1/p3 1]);
    sys_full = tf(num_tilde, den_full);
    
    [y, t_out] = step(sys_full, t_sim);
    
    % Calling the functions
    po_vals(j) = CalculatePercentOvershoot(t_out, y);
    ts_vals(j) = SettlingTime(t_out, y, 0.04);
    tr_vals(j) = RiseTime(t_out, y, 0.1, 0.8);
end

% The plots
figure('Name', 'Problem 3 - Part f Performance Metrics');

% 1. Percent Overshoot (PO)
subplot(3, 1, 1);
plot(p3_range, po_vals, 'b', 'LineWidth', 1.5); 
hold on;
yline(po_tilde, 'r--', 'LineWidth', 1.2);
ylabel('PO (%)'); grid on;
title('Percent Overshoot vs p_3');
legend('Full G(s)', 'Approx \tilde{G}(s)');

% 2. Settling Time (Ts)
subplot(3, 1, 2);
plot(p3_range, ts_vals, 'b', 'LineWidth', 1.5); 
hold on;
yline(ts_tilde, 'r--', 'LineWidth', 1.2);
ylabel('T_s (sec)'); grid on;
title('Settling Time (4%) vs p_3');

% 3. Rise Time (Tr)
subplot(3, 1, 3);
plot(p3_range, tr_vals, 'b', 'LineWidth', 1.5); 
hold on;
yline(tr_tilde, 'r--', 'LineWidth', 1.2);
ylabel('T_r (sec)'); xlabel('p_3'); 
grid on;
title('Rise Time (10%-80%) vs p_3');