%%  L/4 deflection animation
clc
clear
close all

%% Parameters
L   = 0.75;
T   = 250;
rho = 0.005;
c   = sqrt(T/rho);
k   = 0.00635;      % deflection amplitude

%% Eigenvalues
N_max = 15;
for n = 1:N_max
    lambda(n) = (c*n*pi)/L;
    freq(n)   = lambda(n)/(2*pi);
end

%% Initial condition f(x) — triangle at L/4
x = linspace(0, L, 500);

f = zeros(size(x));
for i = 1:length(x)
    if x(i) <= L/4
        f(i) = (4*k/L) * x(i);
    else
        f(i) = (-4*k/(3*L)) * x(i) + (k + k/3);
    end
end

%% Fourier coefficients Bn
% g(x)=0 so Bn* = 0
Bn = zeros(1, N_max);
for n = 1:N_max
    integrand = @(xv) f_func(xv, L, k) .* sin(n*pi*xv/L);
    Bn(n) = (2/L) * integral(integrand, 0, L);
end

%% Time vector — 1 period of mode 1
T1    = 2*pi / lambda(1);
t_vec = linspace(0, T1, 100);

%% N values to show
N_list  = [1, 2, 5, 15];
colors  = [0.85 0.35 0.19;
           0.73 0.46 0.07;
           0.33 0.29 0.72;
           0.11 0.62 0.46];

%% Animation
figh = figure('Position',[100 100 950 700],'Color','white');

for ki = 1:length(t_vec)
    clf
    t_k = t_vec(ki);

    for idx = 1:4
        subplot(2,2,idx)
        N = N_list(idx);

        % Partial sum
        u = zeros(size(x));
        for n = 1:N
            u = u + Bn(n)*cos(lambda(n)*t_k) .* sin(n*pi*x/L);
            % Bn* = 0 since g(x)=0, so only cos term
        end

        % Plot initial shape as reference
        plot(x, f, '--', 'Color',[0.6 0.6 0.6], 'LineWidth',1); hold on
        plot(x, u, 'Color', colors(idx,:), 'LineWidth', 2.5);
        plot(x, zeros(size(x)), 'k-', 'LineWidth', 0.5);
        hold off

        xlim([0 L]); ylim([-k*1.5 k*1.5]);
        xlabel('x (m)'); ylabel('u(x,t) [m]');
        title(sprintf('N = %d terms', N), ...
              'Color', colors(idx,:), 'FontWeight','bold','FontSize',12);
        grid on; box on;
        legend('f(x) initial','u(x,t)','Location','northeast','FontSize',8);
    end

    sgtitle(sprintf('Guitar String — L/4 deflection   t = %.5f s', t_k), ...
            'FontSize',13,'FontWeight','bold');

    movieVector_c(ki) = getframe(figh,[10 10 930 680]);
end

%% Save Part c movie
myWriter = VideoWriter('partc_guitar','MPEG-4');
myWriter.FrameRate = 24;
open(myWriter); writeVideo(myWriter, movieVector_c); close(myWriter);
disp('Part c saved!')

%% Helper function
function val = f_func(xv, L, k)
    val = zeros(size(xv));
    for i = 1:length(xv)
        if xv(i) <= L/4
            val(i) = (4*k/L)*xv(i);
        else
            val(i) = (-4*k/(3*L))*xv(i) + (k + k/3);
        end
    end
end
figure('Position',[100 100 800 450],'Color','white');

N_plot = 20;
freq_plot = zeros(1,N_plot);
Bn_L4 = zeros(1,N_plot);    % L/4 case — numerically computed
Bn_L2 = zeros(1,N_plot);    % L/2 case — analytical formula (Eq.3)

for n = 1:N_plot
    freq_plot(n) = (c*n*pi/L) / (2*pi);   % Hz

    % L/4: use Bn already computed (or recompute)
    integrand = @(xv) f_func(xv, L, k) .* sin(n*pi*xv/L);
    Bn_L4(n) = (2/L) * integral(integrand, 0, L);

    % L/2: analytical formula from Eq.3
    Bn_L2(n) = (32*k*cos(n*pi/2)*sin(n*pi/4)^3) / (n^2*pi^2);
end

stem(freq_plot, abs(Bn_L4), 'filled', 'Color',[0.33 0.29 0.72], ...
     'LineWidth',1.8,'MarkerSize',7,'DisplayName','L/4 deflection'); 
hold on
stem(freq_plot, abs(Bn_L2), 'filled', 'Color',[0.85 0.35 0.19], ...
     'LineWidth',1.8,'MarkerSize',7,'DisplayName','L/2 deflection');
hold off

xlabel('Frequency (Hz)','FontSize',12);
ylabel('|B_n|','FontSize',12);
title('Fourier Coefficients vs Frequency','FontSize',13,'FontWeight','bold');
legend('FontSize',11,'Location','northeast');
grid on 
box on