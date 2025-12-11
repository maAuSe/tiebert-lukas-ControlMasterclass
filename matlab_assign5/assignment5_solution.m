%% Assignment 5 - EKF + LQR for the 2WD Swivel Cart
% High-level script to (1) restate the continuous/discrete models,
% (2) run EKF tuning plots (Q/R sweeps + uncertainty), and
% (3) design/tune the LQR tracker on the discrete error model.
% Data are imported from QRoboticsCenter CSV logs; placeholders are kept
% for runs not yet recorded.

clear; close all; clc;

%% ========================================================================
%  PATHS & DIRECTORIES
%  ========================================================================
projRoot = 'C:\Users\campa\Documents\Arduino';
dataDir = fullfile(projRoot, 'matlab_assign5', 'data');
imgDir  = fullfile(projRoot, 'tex_control', 'ass5_tex', 'images');
if ~exist(imgDir, 'dir'), mkdir(imgDir); end

%% ========================================================================
%  PHYSICAL & GEOMETRIC PARAMETERS (MEASURE ON THE CART)
%  ========================================================================
Ts     = 0.010;     % s (sampling time)
r      = 0.0325;    % m (wheel radius, R_WHEEL)
Lwheel = 0.1660;    % m (wheelbase, WHEELBASE)
a      = Lwheel/2;  % m, half wheelbase (0.083 m)

% Sensor offsets in body frame (X' forward, Y' left); UPDATE after measuring
alpha = 0.075;      % m, front IR along X' from cart center to sensor
beta  = 0.065;      % m, side IR longitudinal offset from center
gamma = 0.078;      % m, side IR lateral offset from center (positive to left)

% Walls p*x + q*y = r. Default: corner at (0,0), cart starts in x<0, y<0.
wall1 = [0, 1, 0];  % y = 0 (front wall W1)
wall2 = [1, 0, 0];  % x = 0 (side wall W2)

%% ========================================================================
%  NOISE COVARIANCES & INITIAL STATE (PLACEHOLDERS TO TUNE)
%  ========================================================================
Q_proc_nom = diag([1e-5, 1e-5, 5e-6]);   % process noise (m^2, m^2, rad^2)
R_meas_nom = diag([1e-4, 1e-4]);         % measurement noise (m^2)
P0_default = diag([0.04, 0.04, (deg2rad(10))^2]); % initial covariance
x0_default = [-0.30; -0.20; 0.0];        % starting pose (-30,-20) cm

%% ========================================================================
%  MODEL DEFINITIONS
%  ========================================================================
% Continuous-time dynamics: xdot = f(x,u)
f_cont = @(x,u) [ u(1) * cos(x(3));
                  u(1) * sin(x(3));
                  u(2) ];

% Discretization (forward Euler)
f_disc = @(x,u) x + Ts * f_cont(x,u);

% Jacobian of discretized dynamics A = df/dx (Euler)
jac_f = @(x,u) [1, 0, -Ts * u(1) * sin(x(3));
                0, 1,  Ts * u(1) * cos(x(3));
                0, 0,  1];

% Measurement model and Jacobian
measFun = @(x) measurementModel(x, alpha, beta, gamma, wall1, wall2);

%% ========================================================================
%  EKF Q/R SWEEP PLACEHOLDERS
%  ========================================================================
ekfRuns = [ ...
  struct('label','Q1_R1','Q',Q_proc_nom,          'R',R_meas_nom,          'file',fullfile(dataDir,'ekf_Q1_R1.csv')); ...
  struct('label','Q5_R1','Q',Q_proc_nom*5,        'R',R_meas_nom,          'file',fullfile(dataDir,'ekf_Q5_R1.csv')); ...
  struct('label','Q1_R5','Q',Q_proc_nom,          'R',R_meas_nom*5,        'file',fullfile(dataDir,'ekf_Q1_R5.csv')); ...
  struct('label','Q5_R5','Q',Q_proc_nom*5,        'R',R_meas_nom*5,        'file',fullfile(dataDir,'ekf_Q5_R5.csv'))  ...
];

ekfNominalIdx = 1;  % choose run to use for uncertainty plots (update after data exist)

%% ========================================================================
%  LQR TRACKER (ERROR DYNAMICS IN CART FRAME)
%  ========================================================================
v_ref   = 0.02;                       % m/s (2 cm/s)
Ad      = [1, 0, 0;
           0, 1, -Ts * v_ref;
           0, 0, 1];
Bd      = [-Ts, 0;
           0,   0;
           0,  -Ts];

Q_lqr = diag([4, 4, 0.8]);            % placeholder state penalty
R_lqr = diag([0.4, 0.4]);             % placeholder input penalty
K_lqr = dlqr(Ad, Bd, Q_lqr, R_lqr);   % 2x3 feedback matrix

lqrRuns = [ ...
  struct('label','LQR-A','Q',Q_lqr,          'R',R_lqr,            'file',fullfile(dataDir,'lqr_A.csv')); ...
  struct('label','LQR-B','Q',Q_lqr*2,        'R',R_lqr,            'file',fullfile(dataDir,'lqr_B.csv')); ...
  struct('label','LQR-C','Q',Q_lqr,          'R',R_lqr*2,          'file',fullfile(dataDir,'lqr_C.csv')); ...
  struct('label','LQR-D','Q',Q_lqr*4,        'R',R_lqr*0.5,        'file',fullfile(dataDir,'lqr_D.csv'))  ...
];

%% ========================================================================
%  EKF DATA PROCESSING (IF FILES AVAILABLE)
%  ========================================================================
statesSweep = struct([]);
for k = 1:numel(ekfRuns)
  if ~isfile(ekfRuns(k).file)
    fprintf('>> Missing EKF run: %s\n', ekfRuns(k).file);
    continue;
  end
  D = readQRC45(ekfRuns(k).file);
  statesSweep(end+1).label = ekfRuns(k).label; %#ok<SAGROW>
  statesSweep(end).t       = D.time;
  statesSweep(end).xhat    = [D.xhat, D.yhat, D.thetahat];
  statesSweep(end).Pdiag   = [D.Pxx, D.Pyy, D.Ptt];
end

if ~isempty(statesSweep)
  plotStateSweep(statesSweep, imgDir);
end

% Uncertainty plots for nominal run
if numel(statesSweep) >= ekfNominalIdx
  Snom = statesSweep(ekfNominalIdx);
  plotUncertainty(Snom, imgDir);
end

%% ========================================================================
%  LQR DATA PROCESSING (TRACKING ERRORS & CONTROL INPUTS)
%  ========================================================================
trackingRuns = struct([]);
for k = 1:numel(lqrRuns)
  if ~isfile(lqrRuns(k).file)
    fprintf('>> Missing LQR run: %s\n', lqrRuns(k).file);
    continue;
  end
  D = readQRC45(lqrRuns(k).file);
  trackingRuns(end+1).label = lqrRuns(k).label; %#ok<SAGROW>
  trackingRuns(end).t       = D.time;
  trackingRuns(end).errs    = [D.x_ref - D.xhat, D.y_ref - D.yhat, wrapToPi(D.theta_ref - D.thetahat)];
  trackingRuns(end).u       = [D.v_ff, D.omega_ff]; % feedforward proxy (replace with u_total if logged)
end

if ~isempty(trackingRuns)
  plotLqrTracking(trackingRuns, imgDir);
end

%% ========================================================================
%  FUNCTIONS
%  ========================================================================
function [z, C] = measurementModel(x, alpha, beta, gamma, w1, w2)
% Returns measurement prediction z = h(x) and Jacobian C = dh/dx.
% w = [p q r] line parameters with px + qy = r.
  theta = x(3);
  cth = cos(theta);
  sth = sin(theta);

  % Sensor positions in world frame
  xf = x(1) + alpha * cth;
  yf = x(2) + alpha * sth;
  xs = x(1) + beta * cth - gamma * sth;
  ys = x(2) + beta * sth + gamma * cth;

  p1 = w1(1); q1 = w1(2); r1 = w1(3);
  p2 = w2(1); q2 = w2(2); r2 = w2(3);
  n1 = hypot(p1, q1);
  n2 = hypot(p2, q2);

  z = zeros(2,1);
  z(1) = (r1 - p1 * xf - q1 * yf) / n1;
  z(2) = (r2 - p2 * xs - q2 * ys) / n2;

  dx1 = -alpha * sth;
  dy1 =  alpha * cth;
  dx2 = -beta * sth - gamma * cth;
  dy2 =  beta * cth - gamma * sth;

  C = [ -p1/n1, -q1/n1, -(p1*dx1 + q1*dy1)/n1;
        -p2/n2, -q2/n2, -(p2*dx2 + q2*dy2)/n2 ];
end

function D = readQRC45(filename)
% Import QRC CSV (Assignment 5 mapping). Skips header rows 1-2.
  opts = detectImportOptions(filename);
  opts.DataLines = [3 inf];
  opts.VariableNamesLine = 2;
  T = readtable(filename, opts);

  getv = @(var, default) pickVar(T, var, default);

  D.time      = (T.Time - T.Time(1)) / 1000; % s
  D.v_ff      = getv('ValueIn0', zeros(height(T),1));
  D.omega_ff  = getv('ValueIn1', zeros(height(T),1));
  D.x_ref     = getv('ValueIn2', zeros(height(T),1));
  D.y_ref     = getv('ValueIn3', zeros(height(T),1));
  D.theta_ref = getv('ValueIn4', zeros(height(T),1));
  D.hasMeas   = getv('ValueIn5', zeros(height(T),1));
  D.speedA    = getv('ValueIn6', zeros(height(T),1));
  D.speedB    = getv('ValueIn7', zeros(height(T),1));
  D.z1        = getv('ValueIn8', zeros(height(T),1));
  D.z2        = getv('ValueIn9', zeros(height(T),1));
  D.voltA     = getv('ValueIn10', zeros(height(T),1));
  D.voltB     = getv('ValueIn11', zeros(height(T),1));
  D.xhat      = getv('ValueIn12', zeros(height(T),1));
  D.yhat      = getv('ValueIn13', zeros(height(T),1));
  D.thetahat  = getv('ValueIn14', zeros(height(T),1));
  D.nu1       = getv('ValueIn15', zeros(height(T),1));
  D.nu2       = getv('ValueIn16', zeros(height(T),1));
  D.Pxx       = getv('ValueIn17', zeros(height(T),1));
  D.Pyy       = getv('ValueIn18', zeros(height(T),1));
  D.Ptt       = getv('ValueIn19', zeros(height(T),1));
end

function out = pickVar(T, name, default)
  if any(strcmp(T.Properties.VariableNames, name))
    out = T.(name);
  else
    out = default;
  end
end

function plotStateSweep(statesSweep, imgDir)
% Overlay x/y/theta estimates for multiple Q/R combinations.
  labels = {statesSweep.label};
  colors = lines(numel(statesSweep));
  fig = figure('Name','EKF State Sweep','Position',[100 100 1100 700]);
  stateNames = {'x_c [m]', 'y_c [m]', '\theta [rad]'};
  for idx = 1:3
    subplot(3,1,idx); hold on; grid on;
    for k = 1:numel(statesSweep)
      plot(statesSweep(k).t, statesSweep(k).xhat(:,idx), 'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', labels{k});
    end
    ylabel(stateNames{idx});
    if idx == 1
      title('EKF state trajectories for different Q/R choices');
    end
    if idx == 3
      xlabel('Time [s]');
      legend('Location','bestoutside');
    end
  end
  exportgraphics(fig, fullfile(imgDir, 'ekf_states_QR_sweep.pdf'), 'ContentType','vector');
end

function plotUncertainty(Snom, imgDir)
% Plot state estimate with 95% CI bands.
  ci = 1.96 * [sqrt(Snom.Pdiag(:,1)), sqrt(Snom.Pdiag(:,2)), sqrt(Snom.Pdiag(:,3))];
  fig = figure('Name','EKF Uncertainty','Position',[100 100 1100 700]);
  stateNames = {'x_c [m]', 'y_c [m]', '\theta [rad]'};
  for idx = 1:3
    subplot(3,1,idx); hold on; grid on;
    fill([Snom.t; flipud(Snom.t)], ...
         [Snom.xhat(:,idx)-ci(:,idx); flipud(Snom.xhat(:,idx)+ci(:,idx))], ...
         [0.8 0.8 0.9], 'EdgeColor','none', 'FaceAlpha',0.4, 'DisplayName','95% CI');
    plot(Snom.t, Snom.xhat(:,idx), 'k', 'LineWidth', 1.5, 'DisplayName','Estimate');
    ylabel(stateNames{idx});
    if idx == 1
      title(sprintf('EKF uncertainty bands (%s)', Snom.label));
    end
    if idx == 3, xlabel('Time [s]'); end
  end
  exportgraphics(fig, fullfile(imgDir, 'ekf_uncertainty_95ci.pdf'), 'ContentType','vector');
end

function plotLqrTracking(trackingRuns, imgDir)
% Plot tracking errors and control signals for LQR tuning sweep.
  colors = lines(numel(trackingRuns));
  % Errors
  fig1 = figure('Name','LQR Errors','Position',[100 100 1100 700]);
  errNames = {'e_x [m]','e_y [m]','e_\theta [rad]'};
  for idx = 1:3
    subplot(3,1,idx); hold on; grid on;
    for k = 1:numel(trackingRuns)
      plot(trackingRuns(k).t, trackingRuns(k).errs(:,idx), 'Color', colors(k,:), 'LineWidth',1.4, ...
        'DisplayName', trackingRuns(k).label);
    end
    ylabel(errNames{idx});
    if idx == 1, title('Tracking errors vs. LQR weights'); end
    if idx == 3, xlabel('Time [s]'); legend('Location','bestoutside'); end
  end
  exportgraphics(fig1, fullfile(imgDir, 'lqr_tracking_errors.pdf'), 'ContentType','vector');

  % Control signals (feedforward proxy)
  fig2 = figure('Name','LQR Inputs','Position',[100 100 1100 500]);
  subplot(2,1,1); hold on; grid on;
  for k = 1:numel(trackingRuns)
    plot(trackingRuns(k).t, trackingRuns(k).u(:,1), 'Color', colors(k,:), 'LineWidth',1.4, ...
      'DisplayName', trackingRuns(k).label);
  end
  ylabel('v [m/s]');
  title('Control signals (feedforward proxy)');
  legend('Location','bestoutside');
  subplot(2,1,2); hold on; grid on;
  for k = 1:numel(trackingRuns)
    plot(trackingRuns(k).t, trackingRuns(k).u(:,2), 'Color', colors(k,:), 'LineWidth',1.4, ...
      'DisplayName', trackingRuns(k).label);
  end
  ylabel('\omega [rad/s]'); xlabel('Time [s]');
  exportgraphics(fig2, fullfile(imgDir, 'lqr_control_signals.pdf'), 'ContentType','vector');
end
