function f_d = eval_f(x,u,Ts)
    %% Parameters
    g    = 9.81;
    m    = 1.4;   % [kg]         Drone mass
    d    = 0.2;   % [m]          Distance motor - drone center
    Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)
    
    % States
    % px = x(1); pz = x(2);
    vx = x(3);
    vz = x(4);
    theta = x(5);
    q = x(6);

    % Controls
    F1 = u(1);
    F2 = u(2);
    
    % Nonlinear, continuous-time model
    f = [vx;
         vz;
         1/m * (F1 + F2) * sin(theta);
         1/m * (F1 + F2) * cos(theta) - g;
         q;
         d/Iy * (F1 - F2)];
   

    %% Discretize with Forward Euler scheme
    f_d = Ts*f + x;
    

end