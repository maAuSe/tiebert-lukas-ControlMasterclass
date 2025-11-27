function [dfdx, dfdu] = eval_jacobians(x,u,Ts)
    %% Parameters
    m    = 1.4;   % [kg]         Drone mass
    d    = 0.2;   % [m]          Distance motor - drone center
    Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)

    theta = x(5);
    q = x(6);
    
    F1 = u(1);
    F2 = u(2);

    
    %% Jacobian wrt states df/dx
    dfdx = zeros(6, 6);
    dfdx(1:2, 3:4) = eye(2);
    dfdx(3, 5) = 1/m * (F1 + F2) * cos(theta); 
    dfdx(4, 5) = -1/m * (F1 + F2) * sin(theta);
    dfdx(5, 6) = 1;

    
    %% Jacobian wrt inputs df/du
    dfdu = zeros(6, 2);
    dfdu(3:4,:) = 1/m * [sin(theta), sin(theta);
                         cos(theta), cos(theta)];
    dfdu(6, :) = d/Iy * [1, -1];


    %% Discretize with Forward Euler scheme
    dfdx = eye(size(dfdx)) + Ts*dfdx;
    dfdu = Ts*dfdu;

end