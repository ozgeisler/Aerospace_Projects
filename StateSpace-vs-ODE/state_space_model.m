clear
clc
close all

%Define A, B, C, D matrices
A = [
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;
    -4 -6 -2 3];

B = [
    0 0;
    0 0;
    0 0;
    3 5];

C = [
    1 0 0 0;
    0 0 1 3;
    0 0 0 0];

D = [
    0 0;
    0 0;
    0 4];

%Define initial condition
x0 = [
    1;      %z(0)
    2;      %zd(0)
    -3;     %zdd(0)
    pi      %zddd(0)
    ];

%Run simulink model
simulinkModel = 'space_state_model.slx';
sim(simulinkModel)