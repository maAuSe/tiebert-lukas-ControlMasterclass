colors.blue   = [0.3010, 0.7450, 0.9330];
colors.red    = [0.6350, 0.0780, 0.1840];
colors.yellow = [0.9290, 0.6940, 0.1250];
colors.green  = [0.1569, 0.7059, 0.2353];
colors.green2  = [0.0569, 0.5059, 0.0353];

set(0, 'DefaultLineLineWidth', 1.5);

disturbance = false;              % Add random wind disturbance.


%% Parameters

params.g    = 9.81;  % [m/s^2]      Gravitational acceleration
params.m    = 1.4;   % [kg]         Drone mass
params.d    = 0.2;   % [m]          Distance motor - drone center
params.Iy   = 0.03;  % [kg*m^2]     Moment of inertia around y-axis (pitch)
params.Fmin = 0.0;   % [N]          Minimum propeller thrust force
params.Fmax = 10.0;  % [N]          Maximum propeller thrust force

% Visualization drone geometry
params.fpz = 0.05;  % [m]
params.r   = 0.075; % [m]

% Added measurement noise
sim.Rmeas = diag([1e-4, 1e-4, 1e-6]);

% Simulation parameters
sim.t = 0:Ts:(N-1)*Ts;
sim.x = zeros(6, N);  % xsim = [px pz vx vz theta q]'
params.Ts = Ts;
params.N = N;

% Initial conditions 
sim.x(1:2, 1) = [1.1 0.53]'; % [px pz]'
sim.x(5, 1)   = 0.13;  % theta

% First measurement
px    = sim.x(1,1);
pz    = sim.x(2,1);
pitch = sim.x(5,1);

y(:,1) = [px, pz, pitch]' + sqrt(sim.Rmeas)*randn(3,1);


%% Reference trajectory
x_pos_ref = 1*cos(2*sim.t);  % trajectory tracking problem
z_pos_ref = 1.5*ones(1,length(sim.t));
vx_ref = gradient(x_pos_ref)/Ts;
vz_ref = gradient(z_pos_ref)/Ts;
xref   = [x_pos_ref; z_pos_ref; vx_ref; vz_ref; zeros(2,N)];

% xref  = repmat(xstar,[1,N]);  % stabilization problem
dxref  = xref - xstar;


%% Nonlinear equations for simulation
% x_dot = f(x,u)
% x = [px, pz, vx, vz, theta,  q]'
%       1   2   3   4   5      6
% Fd : Constant force disturbance
sim.Fd = zeros(2, 1);
if disturbance
    sim.Fd(1:2) = 0.1*randn(2, 1);
end

% state equation
sim.xdot = @(x, u) [x(3);
                    x(4);
                    1/params.m*(u(1)+u(2))*sin(x(5)) + sim.Fd(1);
                    1/params.m*(u(1)+u(2))*cos(x(5)) - params.g + sim.Fd(2);
                    x(6);
                    params.d/params.Iy*(u(1)-u(2))];

simulate = @ (x, u) rk4(sim.xdot, x, max([params.Fmin; params.Fmin], min(u, [params.Fmax; params.Fmax])), params.Ts);  % Including actuator saturation
           
get_meas = @ (x)[x(1), x(2), x(5)]' + sqrt(sim.Rmeas)*randn(3, 1);


%% Feedback controller design
% Linearized & discretized model
% - Jacobians evaluated in xstar, ustar
ctrl.F = zeros(6);  % 'ctrl' struct to make sure this F does not overwrite a possible F created in the exercise.
ctrl.F(1, 3)  = 1;
ctrl.F(2, 4)  = 1;
ctrl.F(3, 5)  = params.g;
ctrl.F(5, 6) = 1;

ctrl.G = zeros(6, 2);
ctrl.G(4, :) = 1/m;
ctrl.G(6, 1) = params.d/params.Iy;
ctrl.G(6, 2) = -params.d/params.Iy;

ctrl.C = zeros(3, 6);
ctrl.C(1:2, 1:2) = eye(2);  % y = [px pz theta]'
ctrl.C(3, 5) = 1;

ctrl.D = 0;

% - Discretization
ctrl.F = Ts*ctrl.F + eye(size(ctrl.F));
ctrl.G = Ts*ctrl.G; 

% LQR
ctrl.Qlqr = diag([1e2, 1e2,... % px pz
                  1e1, 1e1,... % vx vz
                  1e2,...      % theta
                  1e0]);       % q

ctrl.Rlqr = 1*eye(2);

ctrl.Kdlqr = dlqr(ctrl.F, ctrl.G, ctrl.Qlqr, ctrl.Rlqr);
K = ctrl.Kdlqr;
                
