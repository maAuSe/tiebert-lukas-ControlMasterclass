function [dfdx, dfdu] = eval_jacobians(x, u, Ts)
% Jacobians of the discretized 2WD cart model (forward Euler).
% State: x = [x_c; y_c; theta]; Input: u = [v; omega].

    theta = x(3);
    v     = u(1);

    % df/dx of continuous model
    dfdx = [0, 0, -v * sin(theta);
            0, 0,  v * cos(theta);
            0, 0,  0];

    % df/du of continuous model
    dfdu = [cos(theta), 0;
            sin(theta), 0;
            0,          1];

    % Discretize Jacobians (Euler)
    dfdx = eye(3) + Ts * dfdx;
    dfdu = Ts * dfdu;
end
