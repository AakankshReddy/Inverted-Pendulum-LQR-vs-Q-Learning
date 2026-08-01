function [t, X, U, metrics] = simulate_controller(controller_type, K_or_agent, p, x0, T, disturbance)
% SIMULATE_CONTROLLER Runs LQR or the trained Q-learning agent on the
% TRUE NONLINEAR plant (fair comparison - LQR's linear model is only used
% internally to derive K, not to simulate).
%
% controller_type : 'lqr' or 'rl'
% K_or_agent       : LQR gain vector K, or the trained RL agent struct
% x0                : initial state [theta0; theta_dot0]
% T                 : total sim time (s)
% disturbance       : struct with fields .t (time of hit, s), .magnitude (N*m)
%                      set disturbance = [] for no disturbance

    dt = p.dt;
    N = round(T/dt);
    t = (0:N-1)' * dt;
    X = zeros(N, 2);
    U = zeros(N, 1);

    x = x0;
    for k = 1:N
        X(k, :) = x';

        switch controller_type
            case 'lqr'
                u = -K_or_agent * x;
            case 'rl'
                u = rl_policy(K_or_agent, x);
            otherwise
                error('Unknown controller_type');
        end
        u = max(min(u, p.u_max), -p.u_max);
        U(k) = u;

        d = 0;
        if ~isempty(disturbance) && abs(t(k) - disturbance.t) < dt/2
            d = disturbance.magnitude;
        end

        xdot = pendulum_dynamics(x, u, p, d);
        x = x + dt * xdot;
        x(1) = wrap_to_pi_local(x(1));
        x(2) = max(min(x(2), 8), -8);
    end

    metrics = compute_metrics(t, X, U, disturbance);
end

function u = rl_policy(agent, x)
    theta = wrap_to_pi_local(x(1));
    theta_dot = max(min(x(2), 8), -8);
    ti = find(theta >= agent.theta_edges, 1, 'last');
    if isempty(ti), ti = 1; end
    ti = min(ti, numel(agent.theta_edges)-1);
    tdi = find(theta_dot >= agent.theta_dot_edges, 1, 'last');
    if isempty(tdi), tdi = 1; end
    tdi = min(tdi, numel(agent.theta_dot_edges)-1);

    [~, ai] = max(squeeze(agent.Q(ti, tdi, :)));
    u = agent.actions(ai);
end

function w = wrap_to_pi_local(a)
    w = mod(a + pi, 2*pi) - pi;
end

function metrics = compute_metrics(t, X, U, disturbance)
    theta = X(:,1);

    % Steady-state error: mean |theta| over the last 20% of the run
    tail_idx = round(0.8*numel(t)):numel(t);
    metrics.steady_state_error = mean(abs(theta(tail_idx)));

    % Settling time: first time |theta| stays within 2% band (0.02 rad) permanently
    band = 0.02;
    settle_idx = NaN;
    for k = 1:numel(t)
        if all(abs(theta(k:end)) < band)
            settle_idx = k;
            break;
        end
    end
    if isnan(settle_idx)
        metrics.settling_time = NaN; % never settles within band
    else
        metrics.settling_time = t(settle_idx);
    end

    % Overshoot (relative to initial |theta0|)
    metrics.max_abs_theta = max(abs(theta));

    % Control effort (RMS)
    metrics.control_effort_rms = sqrt(mean(U.^2));

    % Disturbance recovery (only if a disturbance was applied)
    if ~isempty(disturbance)
        hit_idx = find(t >= disturbance.t, 1, 'first');
        post = theta(hit_idx:end);
        post_t = t(hit_idx:end) - disturbance.t;
        metrics.peak_deviation_post_disturbance = max(abs(post));
        recover_idx = find(abs(post) < band, 1, 'first');
        if isempty(recover_idx)
            metrics.recovery_time = NaN;
        else
            metrics.recovery_time = post_t(recover_idx);
        end
    end
end
