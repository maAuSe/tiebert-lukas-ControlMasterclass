% Exercise session 5:
% Kalman filter
% 2024
% ====================================


close all
clear variables
addpath(genpath('simulation'))


%% Settings
animation = true;
figures = true;
convergence_consistency = true;
use_estimate = false;


%% Parameters, constants, reference, vectors
g    = 9.81;  % [m/s^2]      Gravitational acceleration
m    = 1.4;   % [kg]         Drone mass
d    = 0.2;   % [m]          Distance motor - drone center
Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)
Fmin = 0.0;
Fmax = 10.0;

Tsim = 5;     % [s]
Ts   = 0.01;  % [s]
N    = Tsim/Ts;

% Dimensions of state, control and measurement vectors
n_x = 6;
n_u = 2;
n_y = 3;

% Equilibrium state
ustar = m*g/2*ones(2, 1);
xstar = [0 1.5 0 0 0 0]';  % Hover at specified altitude.
ystar = [0 1.5 0]';

% Initialization of data structures
u         = zeros(n_u, N);  % [F1 F2]'
u(:, 1)   = ustar;
y         = zeros(n_y, N);  % [px pz theta]'

xhat_lqe = zeros(n_x, N);
xhat_kf  = zeros(n_x, N);
xhat_ekf = zeros(n_x, N);
Phat_lqe = zeros(n_x, n_x, N);
Phat_kf  = zeros(n_x, n_x, N);
Phat_ekf = zeros(n_x, n_x, N);
nu_lqe   = zeros(n_y, N);
nu_kf    = zeros(n_y, N);
nu_ekf   = zeros(n_y, N);
Slqe     = zeros(n_y, n_y, N);
Skf      = zeros(n_y, n_y, N);
Sekf     = zeros(n_y, n_y, N);
Lkf      = zeros(n_x, n_y, N);
Lekf     = zeros(n_x, n_y, N);


% Small signal linearization
dxhat_lqe = xhat_lqe - xstar;
dxhat_kf  = xhat_kf - xstar;
dy    = y - ystar;
du    = u - ustar;

           
%% LQE, KF, EKF design

% Linearized state space model
%     dx_dot = A*dx + B*du , 
%     dx = [dpx, dpz, dvx, dvz, dtheta, dq]'

[A, B] = eval_jacobians_drone(xstar, ustar, Ts);
C = zeros(n_y, n_x);
C(1:2, 1:2) = eye(2);
C(3, 5) = 1;
D = 0; 

% Estimator design
% ----------------
% Process noise covariance and measurement noise covariance
% rho = 1e-4;
rho = 1e-7;
% rho = 1e-9;

Qest = rho*diag([1, 1e-2,...     % px pz
                 1e3,  1e4,...   % vx vz
                 1e-1,...        % theta
                 1e2]);          % q
Rest = diag([1e-4, 1e-4, 1e-6]);
% Rest = 1e2*diag([1e-4, 1e-4, 1e-6]);  % Overestimate measurement noise
% Rest = 1e-2*diag([1e-4, 1e-4, 1e-6]);  % Underestimate measurement noise


% LQE
% ---
Llqe = dlqr(A', A'*C', Qest, Rest)';

% Kalman Filter tuning matrices
% -----------------------------
% Use Qest, Rest - note that you might get better results with 
% different Q for LQE, KF and EKF (they don't all have the same modelling
% errors).


% Initial estimator guess
% -----------------------
xhat_lqe(:, 1)   = xstar;
xhat_kf(:, 1)    = xstar;
xhat_ekf(:, 1)   = xstar;
% Better initial guess
% xhat_lqe(:, 1)   = [1., 0.5, 0., 0., 0.1, 0.]';
% xhat_kf(:, 1)    = [1., 0.5, 0., 0., 0.1, 0.]';
% xhat_ekf(:, 1)   = [1., 0.5, 0., 0., 0.1, 0.]';

dxhat_lqe(:, 1) = xhat_lqe(:, 1) - xstar;
dxhat_kf(:, 1)  = xhat_kf(:, 1) - xstar;

% Initial guess estimation error covariance 
% -----------------------------------------
% Reasonable initial estimate
Phat_lqe(:, :, 1) = diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
Phat_kf(:, :, 1)  = diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
Phat_ekf(:, :, 1) = diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);

% Small initial estimate - overestimate the confidence on actually knowing
% the state: 
% Phat_lqe(:, :, 1) = 1e-3*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
% Phat_kf(:, :, 1)  = 1e-3*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
% Phat_ekf(:, :, 1) = 1e-3*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);

% Large initial estimate
% Phat_lqe(:, :, 1) = 1e2*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
% Phat_kf(:, :, 1)  = 1e2*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);
% Phat_ekf(:, :, 1) = 1e2*diag([0.1, 0.1, 0.01, 0.01, 0.01, 0.01]);



%% Simulation

run simulation/initialize_LQE_KF_EKF.m

for i=2:N
    
    % Plant simulation
    % ----------------
    sim.x(:, i) = simulate(sim.x(:, i-1), u(:, i-1));
  
    % Measurements
    % ------------
    y(:, i) = get_meas(sim.x(:, i));
    dy(:, i) = y(:, i) - ystar;
    
    % Estimator
    % ---------
    % IMPLEMENT YOUR ESTIMATOR HERE!
    % =====================================================================
    % Linear Quadratic Estimator:
    % ---------------------------
    % Estimator prediction
    dxhat_lqe(:, i) = A*dxhat_lqe(:, i-1) + B*du(:, i-1);
    Phat_lqe(:, :, i) = A*Phat_lqe(:, :, i-1)*A' + Qest;
    % Estimator correction
    nu_lqe(:, i) = dy(:, i) - C*dxhat_lqe(:, i);
    Slqe(:, :, i) = C*Phat_lqe(:, :, i)*C' + Rest;
    dxhat_lqe(:, i) = dxhat_lqe(:, i) + Llqe*nu_lqe(:, i);
    Phat_lqe(:, :, i) = (eye(6) - Llqe*C)*Phat_lqe(:, :, i)*(eye(6) - Llqe*C)' + Llqe*Rest*Llqe';

    % Kalman filter:
    % --------------
    % Estimator prediction
    dxhat_kf(:, i) = A*dxhat_kf(:, i-1) + B*du(:, i-1);
    Phat_kf(:, :, i) = A*Phat_kf(:, :, i-1)*A' + Qest;
    % Estimator correction
    nu_kf(:, i) = dy(:, i) - C*dxhat_kf(:, i);
    Skf(:, :, i) = C*Phat_kf(:, :, i)*C' + Rest;
    Lkf(:, :, i) = ((Skf(:, :, i)')\(C*Phat_kf(:, :, i)'))';
    dxhat_kf(:, i) = dxhat_kf(:, i) + Lkf(:, :, i)*nu_kf(:, i);
    Phat_kf(:, :, i) = (eye(6) - Lkf(:, :, i)*C)*Phat_kf(:, :, i);
    
    % Extended kalman filter:
    % -----------------------
    % Jacobian
    dfdx = eval_jacobians_drone(xhat_ekf(:, i-1), u(:, i-1), Ts);
    % Estimator prediction
    xhat_ekf(:, i) = eval_f_drone(xhat_ekf(:, i-1), u(:, i-1), Ts);
    Phat_ekf(:, :, i) = dfdx*Phat_ekf(:, :, i-1)*dfdx' + Qest;
        
    % Estimator correction
    % y = h(x) = C*x, dh/dx = C
    nu_ekf(:, i) = y(:, i) - C*xhat_ekf(:, i);
    Sekf(:, :, i) = C*Phat_ekf(:, :, i)*C' + Rest;
    Lekf(:, :, i) = ((Sekf(:, :, i)')\(C*Phat_ekf(:, :, i)'))';
    xhat_ekf(:, i) = xhat_ekf(:, i) + Lekf(:, :, i)*nu_ekf(:, i);
    Phat_ekf(:, :, i) = (eye(6) - Lekf(:, :, i)*C)*Phat_ekf(:, :, i);
    
    % =====================================================================    
    
    
    % Controller action
    % -----------------
    % Trajectory tracking control law
    if use_estimate
        du(:, i) = - K * (dxhat_lqe(:, i) - dxref(:, i));
%         du(:, i) = - K*(dxhat_kf(:, i) - dxref(:, i));
%         du(:, i) = - K*(xhat_ekf(:, i) - xref(:, i));
        u(:, i) = du(:, i) + ustar;
    else
        dx = sim.x(:, i) - xstar;
        du(:, i) = - K * (dx - dxref(:, i));
        u(:, i) = du(:, i) + ustar;
    end
         
    % Also saturate the control signal here such that the estimator
    % bases its predictions on the 'real' control input.
    u(:, i) = max([Fmin; Fmin], min(u(:, i), [Fmax; Fmax]));
end


% Get estimate from small signal estimate
xhat_lqe = dxhat_lqe + xstar;
xhat_kf = dxhat_kf + xstar;


%% Visualization
if animation
    axislimits = [-2 2 -2 2 0 4];
    animate(sim.x, params, colors, axislimits)
end

if figures
    % Verify that Lkf and Lekf converge towards Llqe - here only for one
    % term, other terms analogous.
    figure()
    hold on
    plot(sim.t, squeeze(Llqe(1, 1).*ones(size(Lkf(1, 1, :)))), "Color", colors.red)
    plot(sim.t, squeeze(Lkf(1, 1, :)), "Color", colors.green)
    plot(sim.t, squeeze(Lekf(1, 1, :)), "Color", colors.green2)
    legend("L_{lqe, 1, 1}", "L_{kf, 1, 1}", "L_{ekf, 1, 1}")
    xlabel("Time (s)");
    
    % Plot the states and measurements
    % p_x (state 1)
    figure()
    hold on
    subplot(2, 3, 1)
    hold on
    plotx = plot(sim.t, y(1, :),'Color', colors.yellow); plotx.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(1, :), "Color", colors.red)
    plot(sim.t, xhat_kf(1, :), "Color", colors.green)
    plot(sim.t, xhat_ekf(1, :), "Color", colors.green2)
    title("x-Position")
    xlabel("Time (s)")
    ylabel("Position (m)")
    legend("x position measurement",...
           "x position LQE estimate", "x position KF estimate", "x position EKF estimate") %, "x-position reference (m)")
    title('x position')
    
    % p_z (state 2)
    subplot(2, 3, 2)
    hold on
    plotz = plot(sim.t, y(2, :), "Color", colors.yellow); plotz.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(2, :), "Color", colors.red)
    plot(sim.t, xhat_kf(2, :), "Color", colors.green)
    plot(sim.t, xhat_ekf(2, :), "Color", colors.green2)
    title("Altitude")
    xlabel("Time (s)")
    ylabel("Altitude (m)")
    legend("Altitude measurement",...
           "Altitude LQE estimate", "Altitude KF estimate", "Altitude EKF estimate")
    title('z-position')

    % v_x (state 3)
    subplot(2, 3, 4)
    hold on
    hold on
    plotvx = plot(sim.t, gradient(y(1, :))/Ts, "Color", colors.yellow); plotvx.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(3, :), "Color", colors.red)
    plot(sim.t, xhat_kf(3, :), "Color", colors.green)
    plot(sim.t, xhat_ekf(3, :), "Color", colors.green2)
    title("x-Velocity")
    xlabel("Time (s)")
    ylabel("Velocity (m/s)")
    legend("v_x finite difference",...
           "v_x LQE estimate", "v_x KF estimate", "v_x EKF estimate")
    title('x-velocity')
        
    
    % v_z (state 4)
    subplot(2, 3, 5)
    hold on
    plotvz = plot(sim.t, gradient(y(2, :))/Ts, "Color", colors.yellow); plotvz.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(4, :), "Color", colors.red)
    plot(sim.t, xhat_kf(4, :), "Color", colors.green)
    plot(sim.t, xhat_ekf(4, :), "Color", colors.green2)
    xlabel("Time (s)")
    ylabel("v_z (m/s)")
    legend("v_z finite difference",...
           "v_z LQE estimate", "v_z KF estimate", "v_z EKF estimate") 
    title('z velocity')  
    
    % pitch (state 5)
    subplot(2, 3, 3)
    hold on
    plotpitch = plot(sim.t, y(3, :), 'Color', colors.yellow); plotpitch.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(5, :), 'Color', colors.red)
    plot(sim.t, xhat_kf(5, :), 'Color', colors.green)
    plot(sim.t, xhat_ekf(5, :), 'Color', colors.green2)
    title("pitch")
    xlabel("Time (s)")
    ylabel("Angle (rad)")
    legend("pitch measurement",...
           "pitch LQE estimate",...
           "pitch KF estimate",...
           "pitch EKF estimate")
    title('pitch')

    % pitch rate (state 6)
    subplot(2, 3, 6)
    hold on
    plotpr = plot(sim.t, gradient(y(3, :))/Ts, "Color", colors.yellow); plotpr.Color(4) = 0.3;
    plot(sim.t, xhat_lqe(6, :), "Color", colors.red)
    plot(sim.t, xhat_kf(6, :), "Color", colors.green)
    plot(sim.t, xhat_ekf(6, :), "Color", colors.green2)
    title("pitch rate")
    xlabel("Time (s)")
    ylabel("Angular rate (rad/s)")
    legend("pitch rate finite difference",...
           "pitch rate LQE estimate", "pitch rate KF estimate", "pitch rate EKF estimate")
    title('pitch rate')
    
    % Control signals
    figure()
    hold on
    plot(sim.t, u(1, :), "Color", colors.red)
    plot(sim.t, u(2, :), "Color", colors.blue)
    xlabel("Time (s)")
    ylabel("Thrust force (N)")
    legend("motor 1", "motor 2")
    
end


%% Analyze convergence & consistency
if convergence_consistency

    % "Small signal" analysis.
    du = u-ustar;
    dy = y-C*xstar;
    dxhat_ekf = xhat_ekf - xstar;
    
    R = repmat(Rest,[1 1 N]);
    
    % Create KalmanExperiment objects.
    ke_lqe = KalmanExperiment(sim.t', dxhat_lqe, Phat_lqe, dy, R, du, nu_lqe, Slqe);
    ke_kf  = KalmanExperiment(sim.t', dxhat_kf,  Phat_kf,  dy, R, du, nu_kf,  Skf);
    ke_ekf = KalmanExperiment(sim.t', dxhat_ekf, Phat_ekf, dy, R, du, nu_ekf, Sekf);

    % Plot "small signal" states with their 95% confidence interval.
    % p_x
    figure()
    subplot(2, 3, 1)
    hold on
    plotstates(ke_lqe, 1, 0.95)
    plotstates(ke_kf, 1, 0.95)
    plotstates(ke_ekf, 1, 0.95)
    % plot(sim.t, sim.x(1, :), "Color", [0 0 0 1])  % This is cheating, because in reality there is no way to know the true state. But here we can use it to compare the estimates with the 'true' states.
    xlabel('Time (s)'); ylabel('\delta p_x (m)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')
    title('x-position')
    
    % p_z
    subplot(2, 3, 2)
    hold on
    plotstates(ke_lqe, 2, 0.95)
    plotstates(ke_kf, 2, 0.95)
    plotstates(ke_ekf, 2, 0.95)
    xlabel('Time (s)'); ylabel('\delta p_z (m)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')    
    title('z-position')
    
    % v_x
    subplot(2, 3, 4)
    hold on
    plotstates(ke_lqe, 3, 0.95)
    plotstates(ke_kf, 3, 0.95)
    plotstates(ke_ekf, 3, 0.95)
    xlabel('Time (s)'); ylabel('\delta v_x (m/s)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')    
    title('x-velocity')

    % v_z
    subplot(2, 3, 5)
    hold on
    plotstates(ke_lqe, 4, 0.95)
    plotstates(ke_kf,  4, 0.95)
    plotstates(ke_ekf, 4, 0.95)
    xlabel('Time (s)'); ylabel('\delta v_z (m/s)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')    
    title('z-velocity')

    % theta pitch
    subplot(2, 3, 3)
    hold on
    plotstates(ke_lqe, 5, 0.95)
    plotstates(ke_kf,  5, 0.95)
    plotstates(ke_ekf, 5, 0.95)
    xlabel('Time (s)'); ylabel('\delta \theta (rad)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')    
    title('pitch')

    % q pitch rate
    subplot(2, 3, 6)
    hold on
    plotstates(ke_lqe, 6, 0.95)
    plotstates(ke_kf,  6, 0.95)
    plotstates(ke_ekf, 6, 0.95)
    xlabel('Time (s)'); ylabel('\delta q (rad/s)');
    legend('LQE, 95% confidence region', 'KF, 95% confidence region', 'EKF, 95% confidence region')    
    title('pitch rate')

    % Consistency
    ke_lqe.analyzeconsistency(); %Figure 7
    ke_kf.analyzeconsistency(); %Figure 8
    ke_ekf.analyzeconsistency(); %Figure 9
    
end

%% Local helper functions for the drone example (decoupled from cart helpers)
function f_d = eval_f_drone(x, u, Ts)
    % Drone model used in the exercise session (6 states).
    g    = 9.81;
    m    = 1.4;   % [kg]         Drone mass
    d    = 0.2;   % [m]          Distance motor - drone center
    Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)
    
    % States
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

    % Discretize with Forward Euler scheme
    f_d = Ts * f + x;
end

function [dfdx, dfdu] = eval_jacobians_drone(x, u, Ts)
    % Jacobians of the drone model used in the exercise session (6 states).
    m    = 1.4;   % [kg]         Drone mass
    d    = 0.2;   % [m]          Distance motor - drone center
    Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)

    theta = x(5);
    
    F1 = u(1);
    F2 = u(2);

    % Jacobian wrt states df/dx
    dfdx = zeros(6, 6);
    dfdx(1:2, 3:4) = eye(2);
    dfdx(3, 5) = 1/m * (F1 + F2) * cos(theta); 
    dfdx(4, 5) = -1/m * (F1 + F2) * sin(theta);
    dfdx(5, 6) = 1;

    % Jacobian wrt inputs df/du
    dfdu = zeros(6, 2);
    dfdu(3:4,:) = 1/m * [sin(theta), sin(theta);
                         cos(theta), cos(theta)];
    dfdu(6, :) = d/Iy * [1, -1];

    % Discretize with Forward Euler scheme
    dfdx = eye(size(dfdx)) + Ts * dfdx;
    dfdu = Ts * dfdu;
end
