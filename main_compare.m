%% main_compare.m
% Compares a white-box LQR controller against a black-box Q-learning
% agent for stabilizing an inverted pendulum, under nominal and
% disturbed conditions. Run this script directly in MATLAB.

clear; clc; close all;
rng(1); % reproducibility

p = pendulum_params();

%% 1. Design white-box controller
[K, A, B] = design_lqr(p);

%% 2. Train black-box controller
fprintf('Training Q-learning agent (this takes ~1-2 min)...\n');
agent = train_qlearning(p);

figure('Name', 'RL Training Curve');
plot(movmean(agent.reward_history, 50), 'LineWidth', 1.5);
xlabel('Episode'); ylabel('Episode reward (smoothed)');
title('Q-learning convergence during training');
grid on;

%% 3. Nominal comparison (no disturbance): convergence speed & steady-state error
x0 = [deg2rad(20); 0];  % start 20 deg off upright
T = 5;

[t_lqr, X_lqr, U_lqr, m_lqr] = simulate_controller('lqr', K, p, x0, T, []);
[t_rl,  X_rl,  U_rl,  m_rl]  = simulate_controller('rl',  agent, p, x0, T, []);

figure('Name', 'Nominal Response');
subplot(2,1,1);
plot(t_lqr, rad2deg(X_lqr(:,1)), 'b', 'LineWidth', 1.5); hold on;
plot(t_rl,  rad2deg(X_rl(:,1)),  'r', 'LineWidth', 1.5);
yline(0, 'k--');
xlabel('Time (s)'); ylabel('\theta (deg)');
legend('LQR (white-box)', 'Q-learning (black-box)');
title('Nominal stabilization from 20\circ offset'); grid on;

subplot(2,1,2);
plot(t_lqr, U_lqr, 'b', 'LineWidth', 1.2); hold on;
plot(t_rl,  U_rl,  'r', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Control torque (N\cdotm)');
legend('LQR', 'RL'); title('Control effort'); grid on;

%% 4. Robustness comparison: impulse disturbance mid-run
x0d = [0; 0]; % start balanced, then hit it
Td = 6;
disturbance.t = 2.0;
disturbance.magnitude = 0.6; % N*m impulse torque

[t_lqr_d, X_lqr_d, U_lqr_d, m_lqr_d] = simulate_controller('lqr', K, p, x0d, Td, disturbance);
[t_rl_d,  X_rl_d,  U_rl_d,  m_rl_d]  = simulate_controller('rl',  agent, p, x0d, Td, disturbance);

figure('Name', 'Disturbance Rejection');
plot(t_lqr_d, rad2deg(X_lqr_d(:,1)), 'b', 'LineWidth', 1.5); hold on;
plot(t_rl_d,  rad2deg(X_rl_d(:,1)),  'r', 'LineWidth', 1.5);
xline(disturbance.t, 'k--', 'Disturbance hit');
xlabel('Time (s)'); ylabel('\theta (deg)');
legend('LQR (white-box)', 'Q-learning (black-box)');
title(sprintf('Recovery from %.2f N\\cdotm impulse disturbance', disturbance.magnitude));
grid on;

%% 5. Metrics table
fprintf('\n================ PERFORMANCE METRICS ================\n');
fprintf('%-30s %12s %12s\n', 'Metric', 'LQR', 'Q-learning');
fprintf('%-30s %12.4f %12.4f\n', 'Settling time (s)', m_lqr.settling_time, m_rl.settling_time);
fprintf('%-30s %12.5f %12.5f\n', 'Steady-state error (rad)', m_lqr.steady_state_error, m_rl.steady_state_error);
fprintf('%-30s %12.4f %12.4f\n', 'Peak |theta| (rad)', m_lqr.max_abs_theta, m_rl.max_abs_theta);
fprintf('%-30s %12.4f %12.4f\n', 'Control effort RMS (N.m)', m_lqr.control_effort_rms, m_rl.control_effort_rms);
fprintf('--- Under disturbance ---\n');
fprintf('%-30s %12.4f %12.4f\n', 'Peak deviation post-hit (rad)', m_lqr_d.peak_deviation_post_disturbance, m_rl_d.peak_deviation_post_disturbance);
fprintf('%-30s %12.4f %12.4f\n', 'Recovery time (s)', m_lqr_d.recovery_time, m_rl_d.recovery_time);
fprintf('=======================================================\n');

%% 6. Save results for the report
results.K = K; results.A = A; results.B = B;
results.metrics_nominal_lqr = m_lqr; results.metrics_nominal_rl = m_rl;
results.metrics_disturbed_lqr = m_lqr_d; results.metrics_disturbed_rl = m_rl_d;
save('comparison_results.mat', 'results', 'agent');
fprintf('Saved results to comparison_results.mat\n');
