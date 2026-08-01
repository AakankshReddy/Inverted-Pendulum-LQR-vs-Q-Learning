function agent = train_qlearning(p)
% TRAIN_QLEARNING Black-box, model-free controller.
% Tabular Q-learning: the agent never sees A, B, m, l, I, b, g explicitly.
% It only observes discretized state transitions and rewards, and learns
% a policy purely from experience (epsilon-greedy exploration).
%
% Reward shaping: raw "upright reward" is extremely sparse near the top,
% so we shape it with a continuous penalty on angle/velocity/effort plus
% a bonus band near upright, which speeds up learning materially.

    %% Discretization grids
    theta_edges     = linspace(-pi, pi, 41);      % 40 bins
    theta_dot_edges = linspace(-8, 8, 41);        % 40 bins
    n_theta     = numel(theta_edges) - 1;
    n_theta_dot = numel(theta_dot_edges) - 1;

    actions = linspace(-p.u_max, p.u_max, 9);     % 9 discrete torque levels
    n_actions = numel(actions);

    Q = zeros(n_theta, n_theta_dot, n_actions);

    %% Hyperparameters
    alpha       = 0.2;     % learning rate
    gamma       = 0.98;    % discount factor
    eps_start   = 1.0;
    eps_end     = 0.05;
    n_episodes  = 4000;
    max_steps   = 400;     % 400 * dt(0.01) = 4s per episode
    dt          = p.dt;

    reward_history = zeros(n_episodes, 1);

    for ep = 1:n_episodes
        % start near the bottom (hardest case: swing-up + stabilize band)
        % for a *stabilization* task (matching the LQR comparison), start
        % within +/- 60 deg of upright with small random velocity.
        theta0     = (rand()*2 - 1) * (pi/3);
        theta_dot0 = (rand()*2 - 1) * 1.0;
        x = [theta0; theta_dot0];

        epsilon = eps_end + (eps_start - eps_end) * exp(-ep / (n_episodes*0.3));
        ep_reward = 0;

        for t = 1:max_steps
            [ti, tdi] = discretize_state(x, theta_edges, theta_dot_edges);

            if rand() < epsilon
                ai = randi(n_actions);
            else
                [~, ai] = max(squeeze(Q(ti, tdi, :)));
            end
            u = actions(ai);

            xdot = pendulum_dynamics(x, u, p, 0);
            x_next = x + dt * xdot;
            x_next(1) = wrap_to_pi(x_next(1));
            x_next(2) = max(min(x_next(2), 8), -8);

            r = reward_shaped(x_next, u);
            ep_reward = ep_reward + r;

            [tni, tdni] = discretize_state(x_next, theta_edges, theta_dot_edges);

            best_next = max(squeeze(Q(tni, tdni, :)));
            Q(ti, tdi, ai) = Q(ti, tdi, ai) + alpha * (r + gamma*best_next - Q(ti, tdi, ai));

            x = x_next;
        end
        reward_history(ep) = ep_reward;

        if mod(ep, 500) == 0
            fprintf('Q-learning episode %d/%d | epsilon=%.3f | avg reward (last 500)=%.1f\n', ...
                ep, n_episodes, epsilon, mean(reward_history(max(1,ep-499):ep)));
        end
    end

    agent.Q = Q;
    agent.actions = actions;
    agent.theta_edges = theta_edges;
    agent.theta_dot_edges = theta_dot_edges;
    agent.reward_history = reward_history;
end

function r = reward_shaped(x, u)
    theta = x(1); theta_dot = x(2);
    r = -(theta^2) - 0.05*(theta_dot^2) - 0.001*(u^2);
    if abs(theta) < 0.05 && abs(theta_dot) < 0.5
        r = r + 1.0; % bonus band near upright
    end
end

function [ti, tdi] = discretize_state(x, theta_edges, theta_dot_edges)
    theta = wrap_to_pi(x(1));
    theta_dot = max(min(x(2), 8), -8);
    ti  = discretize(theta, theta_edges);
    tdi = discretize(theta_dot, theta_dot_edges);
end

function idx = discretize(val, edges)
    idx = find(val >= edges, 1, 'last');
    if isempty(idx), idx = 1; end
    idx = min(idx, numel(edges)-1);
end

function w = wrap_to_pi(a)
    w = mod(a + pi, 2*pi) - pi;
end
