clear; 
clc; 
close all;

A = [0 0 1;
    -1 -6 -6;
     0 0 1];

% Basis for row space 
r1 = [0 0 1];
r2 = [-1 -6 -6];

N = 800;                       % number of samples
alpha = randn(N,1);            % random coefficients
beta  = randn(N,1);

V = alpha.*r1 + beta.*r2;      % N x 3 points in the row space

figure;
scatter3(V(:,1), V(:,2), V(:,3), 10, 'filled');
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Row Space of A (random linear combinations)');

% optional: draw some vectors from origin
hold on;
K = 30;
quiver3(zeros(K,1),zeros(K,1),zeros(K,1), V(1:K,1),V(1:K,2),V(1:K,3), 0, 'LineWidth', 1)

% Basis for column space (from part b)
c1 = [0; -1; 0];
c3 = [1; -6; 1];


V = alpha.*c1.' + beta.*c3.';  % make them row for stacking -> N x 3

figure;
scatter3(V(:,1), V(:,2), V(:,3), 10, 'filled');
grid on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Column Space of A (random linear combinations)');

hold on;
K = 30;
quiver3(zeros(K,1),zeros(K,1),zeros(K,1), V(1:K,1),V(1:K,2),V(1:K,3), 0, 'LineWidth', 1);
