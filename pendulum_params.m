function p = pendulum_params()
% PENDULUM_PARAMS Physical parameters for a torque-driven inverted pendulum.
%
% Model (theta measured from the UPRIGHT unstable equilibrium, theta = 0):
%   I*theta_ddot = m*g*l*sin(theta) - b*theta_dot + u
%
% where u is the applied torque (control input).
%
% Linearized about theta = 0 (small angle, sin(theta) ~ theta):
%   theta_ddot = (m*g*l/I)*theta - (b/I)*theta_dot + (1/I)*u
%
%   x = [theta; theta_dot],  xdot = A*x + B*u
%   A = [0, 1; m*g*l/I, -b/I]
%   B = [0; 1/I]

    p.m = 0.2;      % pendulum bob mass (kg)
    p.l = 0.3;      % length to center of mass (m)
    p.I = 0.006;    % moment of inertia about pivot (kg*m^2)  (~ m*l^2 + rod term)
    p.b = 0.01;     % viscous friction coefficient (N*m*s/rad)
    p.g = 9.81;     % gravity (m/s^2)

    p.u_max = 1.5;  % actuator torque saturation (N*m) - applies to BOTH controllers
    p.dt    = 0.01; % simulation timestep (s)
end
