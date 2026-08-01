function xdot = pendulum_dynamics(x, u, p, d)
% PENDULUM_DYNAMICS Nonlinear equations of motion (ground truth plant).
%
% x = [theta; theta_dot], u = applied torque, p = params struct
% d = external disturbance torque (N*m), default 0
%
% Both the LQR and the RL agent act on THIS nonlinear plant - LQR uses a
% *linearized* internal model to derive its gain, but is tested here on
% the true nonlinear system, which is the whole point of the comparison.

    if nargin < 4
        d = 0;
    end

    theta     = x(1);
    theta_dot = x(2);

    u = max(min(u, p.u_max), -p.u_max); % actuator saturation

    theta_ddot = (p.m * p.g * p.l * sin(theta) - p.b * theta_dot + u + d) / p.I;

    xdot = [theta_dot; theta_ddot];
end
