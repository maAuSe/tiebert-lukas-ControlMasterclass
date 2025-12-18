function x_next = eval_f(x, u, Ts)
% Discrete-time forward Euler update for the 2WD cart pose.
% State: x = [x_c; y_c; theta]; Input: u = [v; omega].

    theta = x(3);
    v     = u(1);
    omega = u(2);

    % Continuous-time model: xdot = [v cos(theta); v sin(theta); omega]
    xdot = [v * cos(theta);
            v * sin(theta);
            omega];

    % Forward Euler discretization
    x_next = x + Ts * xdot;
end
