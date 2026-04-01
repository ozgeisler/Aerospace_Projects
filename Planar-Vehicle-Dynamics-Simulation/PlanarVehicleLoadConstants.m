function [GEOMETRY, PROPULSION, SIMULATION, ENVIRONMENT, INPUTS] = PlanarVehicleLoadConstants(vehicle)
if strcmp(vehicle, 'Boat')
    % Geometry
    GEOMETRY.m  = 10;    % kg
    GEOMETRY.Ib = 5;     % kgm2

    % Propulsion
    PROPULSION.Fmax = 10;   % N
    PROPULSION.Mmax = 0.5;  % Nm

    % Environment
    ENVIRONMENT.CT = 0.025;
    ENVIRONMENT.CR = 0.75;

    % Simulation
    SIMULATION.X_0 = [200; 200; 0; 0; 0; 0];

    % Inputs
    INPUTS.type = 'step';
end
end