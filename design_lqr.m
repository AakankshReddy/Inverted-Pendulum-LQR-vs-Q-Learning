function [K, A, B] = design_lqr(p)
% DESIGN_LQR White-box, model-based controller.
% Linearizes the pendulum about the upright equilibrium and solves the
% continuous-time algebraic Riccati equation for the optimal gain K such
% that u = -K*x minimizes  J = integral( x'Qx + u'Ru ) dt.
%
% Requires the Control System Toolbox (lqr function).

    A = [0, 1;
         p.m*p.g*p.l/p.I, -p.b/p.I];
    B = [0; 1/p.I];

    % Tuning weights: penalize angle error more than angular velocity,
    % and keep control effort moderate (this is the "hand tuning" that
    % represents the human-in-the-loop cost of the white-box approach).
    Q = diag([50, 1]);
    R = 0.5;

    K = lqr(A, B, Q, R);

    fprintf('--- LQR Design ---\n');
    fprintf('A =\n'); disp(A);
    fprintf('B =\n'); disp(B);
    fprintf('K = [%.4f, %.4f]\n', K(1), K(2));

    eigA_cl = eig(A - B*K);
    fprintf('Closed-loop eigenvalues: %.3f%+.3fi, %.3f%+.3fi\n\n', ...
        real(eigA_cl(1)), imag(eigA_cl(1)), real(eigA_cl(2)), imag(eigA_cl(2)));
end
