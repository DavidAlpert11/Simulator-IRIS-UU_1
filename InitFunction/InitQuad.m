function Quad = InitQuad
%UNTITLED19 Summary of this function goes here
%   Detailed explanation goes here
% Wil Selby
% Washington, DC
% May 30, 2015

% Quadrotor Physical Parameters
Quad.m = 1.4;      % Quadrotor mass (kg)
Quad.l = .56;     % Distance from the center of mass to the each motor (m)
Quad.t = .02;   %Thickness of the quadrotor's arms for drawing purposes (m)
Quad.rot_rad = .1;   %Radius of the propellor (m)
Quad.Kd = 1.3858e-6;    % Drag torque coeffecient (kg-m^2)

Quad.Kdx = 0.16481;    % Translational drag force coeffecient (kg/s)
Quad.Kdy = 0.31892;    % Translational drag force coeffecient (kg/s)
Quad.Kdz = 1.1E-6;    % Translational drag force coeffecient (kg/s)

Quad.Jx = .05;     % Moment of inertia about X axis (kg-m^2)
Quad.Jy = .05;     % Moment of inertia about Y axis (kg-m^2)
Quad.Jz = .24;    % Moment of inertia about Z axis (kg-m^2)

% Motor Parameters
Quad.KT = 1.3328e-5;    % Thrust force coeffecient (kg-m)
Quad.Jp = 0.044;     % Moment of Intertia of the rotor (kg-m^2)


%%
end

