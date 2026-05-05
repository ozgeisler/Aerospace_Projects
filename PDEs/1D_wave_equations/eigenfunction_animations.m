clc
clear 
close all

% Define The constants

L = 0.75;   % length
T = 250;    % tension
rho = 0.005; %density
c = sqrt(T/rho);
n_modes = 4;
amp = 0.00635;

for n=1:n_modes
    lambda(n) = (c * n* pi)/L;
    Bn(n) = (8*k*(4*sin((n*pi)/4)-sin(n*pi)))/(3*n^2*pi^2);
    Bn2(n) = 32*amp*cos((n*pi)/4)*(sin((n*pi)/4)^3)/(n^2*pi^2)
end
T1 = 2*pi/lambda(1); % Period

x = linspace(0,0.75,200);
t_vec = linspace(0,T1,200);

figure
for k=1:length(t_vec)
    clf
    t_k = t_vec(k);
    for n = 1:n_modes
        subplot(2,2,n)
        
        u_n = (Bn(n)* cos(lambda(n)*t_k) + Bn(n) * sin(lambda(n)*t_k)) ...
              .* sin(n*pi*x/L);
        u_n2 = (Bn2(n)* cos(lambda(n)*t_k) + 0 * sin(lambda(n)*t_k)) ...
              .* sin(n*pi*x/L);
           plot(x, u_n, 'LineWidth', 2.5)
           hold on
           plot(x,u_n2)
    end
 movieVector(k) = getframe; 
end


myWriter = VideoWriter('guitar_string', 'MPEG-4');
myWriter.FrameRate = 30;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);

disp('Done! guitar_string.mp4 saved.')


%% Step 3: Save the Movie
myWriter = VideoWriter('animation', 'MPEG-4');
myWriter.FrameRate = 30;

open(myWriter);
writeVideo(myWriter, movieVector);
close(myWriter);

disp('Done! guitar_string.mp4 saved.')