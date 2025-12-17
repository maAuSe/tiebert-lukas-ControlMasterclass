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
r      = 0.0330;    % m (wheel radius, R_WHEEL, per Assignment 3 measurement)
Lwheel = 0.1660;    % m (wheelbase, WHEELBASE)
a      = Lwheel/2;  % m, half wheelbase (0.083 m)

% Sensor offsets in body frame (X' forward, Y' left); UPDATE after measuring
alpha = 0.075;      % m, front IR along X' from cart center to sensor
beta  = 0.065;      % m, side IR longitudinal offset from center
gamma = 0.078;      % m, side IR lateral offset from center (positive to left)

% Walls p*x + q*y = r. Default: corner at (0,0), cart starts in x<0, y<0.
wall1 = [0, 1, 0];  % y = 0 (front wall W1)
wall2 = [1, 0, 0];  % x = 0 (side wall W2)
geom  = struct('alpha',alpha,'beta',beta,'gamma',gamma,'wall1',wall1,'wall2',wall2);

%% ========================================================================
%  NOISE COVARIANCES & INITIAL STATE (PLACEHOLDERS TO TUNE)
%  ========================================================================
% Nominal EKF covariances (must match Arduino firmware: arduino_files/CT-EKF-Swivel/extended_kalman_filter.cpp)
% Set per attached spec: diag(1e-8, 1e-8, 1e-8) in SI units.
Q_proc_base = diag([1e-8, 1e-8, 1e-8]);   % process noise (m^2, m^2, rad^2)
R_meas_base = diag([1e-8, 1e-8]);         % measurement noise (m^2)

firmware_Q_scale = 1.0;      % keep unity so MATLAB/Arduino match exactly
firmware_R_scale = 1.0;

Q_proc_nom = Q_proc_base * firmware_Q_scale;
R_meas_nom = R_meas_base * firmware_R_scale;
P0_default = diag([1e-8, 1e-8, 1e-8]); % must match resetKalmanFilter() on Arduino
x0_default = [-0.30; -0.20; 0.0];        % starting pose (-30,-20) cm
R_meas_off_factor = 1e6;                 % inflate R when sensors are disabled

%% ========================================================================
%  MODEL DEFINITIONS
%  ========================================================================
% Motor-side relation (spec 1a): v = r/2*(wR + wL), omega = r/L*(wR - wL)
wheelSpeedsFromVOmega = @(v, omega) [v/r + (Lwheel/(2*r)) * omega;   % right wheel speed [rad/s]
                                     v/r - (Lwheel/(2*r)) * omega];  % left  wheel speed [rad/s]

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

% Rotation matrix for world -> body error conversion (spec 4a)
rotWorldToBody = @(theta) [ cos(theta),  sin(theta), 0;
                           -sin(theta),  cos(theta), 0;
                            0,           0,          1];

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
ekfExps = struct([]);
% Tip (spec 3b): for manual imports you can also call KalmanExperiment.createfromQRC45().
for k = 1:numel(ekfRuns)
  if ~isfile(ekfRuns(k).file)
    fprintf('>> Missing EKF run: %s\n', ekfRuns(k).file);
    continue;
  end
  [ke, raw] = loadEkfRun(ekfRuns(k), geom, R_meas_off_factor);
  ekfExps(end+1).label = ekfRuns(k).label; %#ok<SAGROW>
  ekfExps(end).ke      = ke;
  ekfExps(end).raw     = raw;
end

if ~isempty(ekfExps)
  plotEkfStateSweep(ekfExps, imgDir, 0.95);
end

% Uncertainty + measurement plots for nominal run
if numel(ekfExps) >= ekfNominalIdx
  keNom = ekfExps(ekfNominalIdx).ke;
  plotEkfNominalBands(keNom, imgDir, 0.95);
  plotEkfMeasurements(keNom, imgDir, 0.95);
end

%% ========================================================================
%  OPTIONAL: OFFLINE EKF RE-RUN (RETROACTIVE Q/R SWEEPS ON ONE LOG)
%  ========================================================================
% This section re-runs the EKF in MATLAB using the logged inputs/measurements,
% so you can study the effect of Q/R without reflashing the Arduino.
% It only plots theta-hat trajectories (per your request).
doOfflineEkfSweeps = true;
offlineEkfFile = fullfile(dataDir, 'ekf_Q1_R1.csv');
if doOfflineEkfSweeps && isfile(offlineEkfFile)
  Doff = parseQrcAssignment5(offlineEkfFile);

  % Wider sweeps (6 orders of magnitude) for independent Q and R scaling
  qScales = logspace(-3, 3, 13); % 1e-3 ... 1e3
  rScales = logspace(-3, 3, 13); % 1e-3 ... 1e3

  xHat_Q     = zeros(numel(qScales), numel(Doff.time));
  yHat_Q     = zeros(numel(qScales), numel(Doff.time));
  thetaHat_Q = zeros(numel(qScales), numel(Doff.time));
  xHat_R     = zeros(numel(rScales), numel(Doff.time));
  yHat_R     = zeros(numel(rScales), numel(Doff.time));
  thetaHat_R = zeros(numel(rScales), numel(Doff.time));

  for i = 1:numel(qScales)
    xhatOff = rerunEkfOffline(Doff, Ts, geom, x0_default, P0_default, Q_proc_nom * qScales(i), R_meas_nom);
    xHat_Q(i,:)     = xhatOff(1,:);
    yHat_Q(i,:)     = xhatOff(2,:);
    thetaHat_Q(i,:) = xhatOff(3,:);
  end

  for i = 1:numel(rScales)
    xhatOff = rerunEkfOffline(Doff, Ts, geom, x0_default, P0_default, Q_proc_nom, R_meas_nom * rScales(i));
    xHat_R(i,:)     = xhatOff(1,:);
    yHat_R(i,:)     = xhatOff(2,:);
    thetaHat_R(i,:) = xhatOff(3,:);
  end

  % -- Q sweep plots (x, y, theta)
  figure('Name', 'Offline EKF: x sweep vs Q scale', 'Position', [160 140 900 420]);
  hold on; grid on;
  cmap = lines(numel(qScales));
  for i = 1:numel(qScales)
    plot(Doff.time, xHat_Q(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('Q x %.2g', qScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{x} [m]');
  title('Offline EKF re-run: effect of Q scaling on \hat{x} (R fixed)');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: y sweep vs Q scale', 'Position', [170 150 900 420]);
  hold on; grid on;
  cmap = lines(numel(qScales));
  for i = 1:numel(qScales)
    plot(Doff.time, yHat_Q(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('Q x %.2g', qScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{y} [m]');
  title('Offline EKF re-run: effect of Q scaling on \hat{y} (R fixed)');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: \theta sweep vs Q scale', 'Position', [180 160 900 420]);
  hold on; grid on;
  cmap = lines(numel(qScales));
  for i = 1:numel(qScales)
    plot(Doff.time, thetaHat_Q(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('Q x %.2g', qScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{\theta} [rad]');
  title('Offline EKF re-run: effect of Q scaling on \hat{\theta} (R fixed)');
  legend('Location', 'bestoutside');

  % -- R sweep plots (x, y, theta)
  figure('Name', 'Offline EKF: x sweep vs R scale', 'Position', [160 180 900 420]);
  hold on; grid on;
  cmap = lines(numel(rScales));
  for i = 1:numel(rScales)
    plot(Doff.time, xHat_R(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('R x %.2g', rScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{x} [m]');
  title('Offline EKF re-run: effect of R scaling on \hat{x} (Q fixed)');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: y sweep vs R scale', 'Position', [170 200 900 420]);
  hold on; grid on;
  cmap = lines(numel(rScales));
  for i = 1:numel(rScales)
    plot(Doff.time, yHat_R(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('R x %.2g', rScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{y} [m]');
  title('Offline EKF re-run: effect of R scaling on \hat{y} (Q fixed)');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: \theta sweep vs R scale', 'Position', [180 200 900 420]);
  hold on; grid on;
  cmap = lines(numel(rScales));
  for i = 1:numel(rScales)
    plot(Doff.time, thetaHat_R(i,:), 'Color', cmap(i,:), 'DisplayName', sprintf('R x %.2g', rScales(i)));
  end
  xlabel('Time [s]'); ylabel('\hat{\theta} [rad]');
  title('Offline EKF re-run: effect of R scaling on \hat{\theta} (Q fixed)');
  legend('Location', 'bestoutside');

  % -- Joint Q/R sweep (vary both simultaneously over a grid)
  qrScales = logspace(-3, 3, 5); % coarser grid to keep plots readable
  qrPairs = struct('q', {}, 'r', {});
  for iq = 1:numel(qrScales)
    for ir = 1:numel(qrScales)
      qrPairs(end+1).q = qrScales(iq); %#ok<SAGROW>
      qrPairs(end).r   = qrScales(ir);
    end
  end

  xHat_QR = cell(numel(qrPairs),1);
  yHat_QR = cell(numel(qrPairs),1);
  thetaHat_QR = cell(numel(qrPairs),1);
  for k = 1:numel(qrPairs)
    xhatOff = rerunEkfOffline(Doff, Ts, geom, x0_default, P0_default, Q_proc_nom * qrPairs(k).q, R_meas_nom * qrPairs(k).r);
    xHat_QR{k}     = xhatOff(1,:);
    yHat_QR{k}     = xhatOff(2,:);
    thetaHat_QR{k} = xhatOff(3,:);
  end

  colorsQR = lines(numel(qrPairs));
  figure('Name', 'Offline EKF: x sweep vs Q/R grid', 'Position', [150 220 900 420]);
  hold on; grid on;
  for k = 1:numel(qrPairs)
    plot(Doff.time, xHat_QR{k}, 'Color', colorsQR(k,:), 'DisplayName', sprintf('Q x %.2g, R x %.2g', qrPairs(k).q, qrPairs(k).r));
  end
  xlabel('Time [s]'); ylabel('\hat{x} [m]');
  title('Offline EKF re-run: \hat{x} trajectories for joint Q/R scaling');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: y sweep vs Q/R grid', 'Position', [160 240 900 420]);
  hold on; grid on;
  for k = 1:numel(qrPairs)
    plot(Doff.time, yHat_QR{k}, 'Color', colorsQR(k,:), 'DisplayName', sprintf('Q x %.2g, R x %.2g', qrPairs(k).q, qrPairs(k).r));
  end
  xlabel('Time [s]'); ylabel('\hat{y} [m]');
  title('Offline EKF re-run: \hat{y} trajectories for joint Q/R scaling');
  legend('Location', 'bestoutside');

  figure('Name', 'Offline EKF: theta sweep vs Q/R grid', 'Position', [170 260 900 420]);
  hold on; grid on;
  for k = 1:numel(qrPairs)
    plot(Doff.time, thetaHat_QR{k}, 'Color', colorsQR(k,:), 'DisplayName', sprintf('Q x %.2g, R x %.2g', qrPairs(k).q, qrPairs(k).r));
  end
  xlabel('Time [s]'); ylabel('\hat{\theta} [rad]');
  title('Offline EKF re-run: \hat{\theta} trajectories for joint Q/R scaling');
  legend('Location', 'bestoutside');
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
  D = parseQrcAssignment5(lqrRuns(k).file);
  trackingRuns(end+1).label = lqrRuns(k).label; %#ok<SAGROW>
  trackingRuns(end).t       = D.time;
  err_world                  = [D.x_ref - D.xhat, D.y_ref - D.yhat, wrapToPiLocal(D.theta_ref - D.thetahat)];
  err_body                   = zeros(size(err_world));
  for i = 1:numel(D.time)
    err_body(i,:) = (rotWorldToBody(D.thetahat(i)) * err_world(i,:).').';
  end
  trackingRuns(end).err_world = err_world;
  trackingRuns(end).err_body  = err_body;
  trackingRuns(end).u         = [D.v, D.omega];
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

function [ke, raw] = loadEkfRun(runCfg, geom, R_off_factor)
% Build a KalmanExperiment from a QRC CSV (assignment 5 mapping).
  D = parseQrcAssignment5(runCfg.file);
  N = numel(D.time);
  xhat = [D.xhat, D.yhat, D.thetahat]';
  y    = [D.z1, D.z2]';
  u    = [D.v, D.omega]';
  hasMeas = D.hasMeas ~= 0;

  if ~any(hasMeas)
    warning('loadEkfRun:NoMeasurements', 'No measurements enabled in %s (hasMeas is always 0). EKF will be effectively open-loop and measurement plots will be empty.', runCfg.file);
  end

  P = zeros(3,3,N);
  for i = 1:N
    P(:,:,i) = diag([D.Pxx(i), D.Pyy(i), D.Ptt(i)]);
  end

  Rall = repmat(runCfg.R, [1 1 N]);
  idxNoMeas = find(~hasMeas);
  if ~isempty(idxNoMeas)
    Rall(:,:,idxNoMeas) = repmat(runCfg.R * R_off_factor, [1 1 numel(idxNoMeas)]); % mimic sensor drop-out
  end
  y(:, ~hasMeas) = NaN;                         % show gaps when sensors are disabled

  % Innovation covariance for plotting/consistency
  S = zeros(2,2,N);
  nu = zeros(2,N);
  for i = 1:N
    if hasMeas(i)
      [zPred, Ck] = measurementModel(xhat(:,i), geom.alpha, geom.beta, geom.gamma, geom.wall1, geom.wall2);
    else
      Ck = zeros(2,3);
      zPred = zeros(2,1);
    end
    if hasMeas(i)
      nu(:,i) = y(:,i) - zPred;
    end
    S(:,:,i) = Ck * P(:,:,i) * Ck' + Rall(:,:,i);
  end

  ke = KalmanExperiment(D.time, xhat, P, y, Rall, u, nu, S);
  raw = struct('t', D.time, 'xhat', xhat, 'Pdiag', [D.Pxx, D.Pyy, D.Ptt], ...
               'hasMeas', hasMeas, 'y', y, 'u', u);
end

function D = parseQrcAssignment5(filename)
% Parse a QRoboticsCenter CSV exported for assignment 5.
  opts = detectImportOptions(filename);
  opts.DataLines = [3 inf];
  opts.VariableNamesLine = 2;
  T = readtable(filename, opts);
  N = height(T);
  getv = @(names, default) pickVar(T, names, default, N);

  tRaw = T.Time;
  tRaw = tRaw(:);
  tRel = tRaw - tRaw(1);
  dtMed = median(diff(tRel), 'omitnan');
  if ~isfinite(dtMed)
    dtMed = NaN;
  end

  if ~isnan(dtMed) && dtMed > 500
    D.time = tRel / 1e6;
  elseif ~isnan(dtMed) && dtMed > 0.5
    D.time = tRel / 1000;
  else
    D.time = tRel;
  end
  D.time = D.time(:);
  D.v_ff      = getv({'ValueIn0','v_ff'}, zeros(N,1));
  D.omega_ff  = getv({'ValueIn1','omega_ff'}, zeros(N,1));
  D.z1        = getv({'ValueIn2','z1','y1'}, NaN(N,1));
  D.z2        = getv({'ValueIn3','z2','y2'}, NaN(N,1));
  D.xhat      = getv({'ValueIn4','xhat','x1'}, zeros(N,1));
  D.yhat      = getv({'ValueIn5','yhat','x2'}, zeros(N,1));
  D.thetahat  = getv({'ValueIn6','thetahat','x3'}, zeros(N,1));
  D.Pxx       = getv({'ValueIn7','Pxx','P11'}, 1e-6 * ones(N,1));
  D.Pyy       = getv({'ValueIn8','Pyy','P22'}, 1e-6 * ones(N,1));
  D.Ptt       = getv({'ValueIn9','Ptt','P33'}, 1e-6 * ones(N,1));
  D.v         = getv({'ValueIn10','v','v_applied'}, D.v_ff);
  D.omega     = getv({'ValueIn11','omega','omega_applied'}, D.omega_ff);

  D.hasMeas = isfinite(D.z1) & isfinite(D.z2);

  x0 = -0.30;
  y0 = -0.20;
  theta0 = 0.0;
  [D.x_ref, D.y_ref, D.theta_ref] = reconstructReferenceFromVOmega(D.time, D.v_ff, D.omega_ff, x0, y0, theta0);
end

function [x_ref, y_ref, theta_ref] = reconstructReferenceFromVOmega(t, v, omega, x0, y0, theta0)
  t = t(:);
  v = v(:);
  omega = omega(:);
  N = numel(t);

  x_ref = zeros(N,1);
  y_ref = zeros(N,1);
  theta_ref = zeros(N,1);

  x_ref(1) = x0;
  y_ref(1) = y0;
  theta_ref(1) = theta0;

  if N < 2
    return;
  end

  dt = diff(t);
  dt(~isfinite(dt)) = 0;
  dt(dt < 0) = 0;
  dt = min(dt, 0.1);

  for k = 2:N
    theta_ref(k) = theta_ref(k-1) + dt(k-1) * omega(k-1);
    x_ref(k) = x_ref(k-1) + dt(k-1) * v(k-1) * cos(theta_ref(k-1));
    y_ref(k) = y_ref(k-1) + dt(k-1) * v(k-1) * sin(theta_ref(k-1));
  end

  theta_ref = wrapToPiLocal(theta_ref);
end

function th = wrapToPiLocal(th)
  th = mod(th + pi, 2*pi) - pi;
end

function xhat = rerunEkfOffline(D, Ts, geom, x0, P0, Qk, Rk)
% Re-run the same EKF structure as the Arduino firmware, using logged u/y.
% Returns xhat as 3xN (x,y,theta).

  N = numel(D.time);
  xhat = zeros(3, N);
  x = x0(:);
  P = P0;

  for k = 1:N
    if k > 1
      u = [D.v(k-1); D.omega(k-1)];
      [x, P] = ekfPredictEuler(x, P, u, Ts, Qk);
    end

    if isfinite(D.z1(k)) && isfinite(D.z2(k))
      y = [D.z1(k); D.z2(k)];
      [x, P] = ekfCorrectWallRanges(x, P, y, geom, Rk);
    end

    x(3) = wrapToPiLocal(x(3));
    xhat(:,k) = x;
  end
end

function [x, P] = ekfPredictEuler(x, P, u, Ts, Qk)
  v = u(1);
  omega = u(2);
  th = x(3);

  cth = cos(th);
  sth = sin(th);

  % x_{k+1|k} = x_k + Ts * f(x_k,u_k)
  x = x + Ts * [v * cth; v * sth; omega];

  % A = I + Ts * df/dx
  A = [1, 0, -Ts * v * sth;
       0, 1,  Ts * v * cth;
       0, 0,  1];

  P = A * P * A' + Qk;
end

function [x, P] = ekfCorrectWallRanges(x, P, y, geom, Rk)
  [zPred, C] = measurementModel(x, geom.alpha, geom.beta, geom.gamma, geom.wall1, geom.wall2);
  nu = y - zPred;
  S = C * P * C' + Rk;
  K = P * C' / S;
  x = x + K * nu;
  P = P - K * C * P;
end

function out = pickVar(T, names, default, N)
% Return first matching column in table T for given candidate names.
  if ~iscell(names); names = {names}; end
  out = [];
  for i = 1:numel(names)
    if any(strcmp(T.Properties.VariableNames, names{i}))
      out = T.(names{i});
      break;
    end
  end
  if isempty(out)
    out = default;
  end
  if isscalar(out)
    out = out * ones(N,1);
  elseif isvector(out)
    out = out(:);
  end
end

function plotEkfStateSweep(ekfExps, imgDir, confidence)
% Per-state overlay of EKF estimates for different Q/R choices using plotstates.
  if nargin < 3, confidence = 0.95; end
  colors = lines(numel(ekfExps));
  stateNames = {'x_c [m]', 'y_c [m]', '\theta [rad]'};
  for idx = 1:3
    fig = figure('Name', sprintf('EKF state %d sweep', idx), 'Position', [100 100 850 500]);
    hold on; grid on;
    for k = 1:numel(ekfExps)
      plotstates(ekfExps(k).ke, idx, confidence);
      hLine = findobj(gca, 'Type','line', 'DisplayName', sprintf('state %d (%.0f%% interval)', idx, confidence*100));
      hLine(1).Color = colors(k,:);
      hLine(1).DisplayName = ekfExps(k).label;
    end
    ylabel(stateNames{idx});
    xlabel('Time [s]');
    legend('Location','bestoutside');
    title('EKF state trajectories vs Q/R');
    exportgraphics(fig, fullfile(imgDir, sprintf('ekf_state%d_QR_sweep.pdf', idx)), 'ContentType','vector');
  end
end

function plotEkfNominalBands(keNom, imgDir, confidence)
% Save 95% confidence plots for the nominal EKF run (states).
  if nargin < 3, confidence = 0.95; end
  stateNames = {'x_c [m]', 'y_c [m]', '\theta [rad]'};
  for idx = 1:3
    fig = figure('Name', sprintf('EKF state %d CI', idx), 'Position', [120 120 800 400]);
    hold on; grid on;
    plotstates(keNom, idx, confidence);
    ylabel(stateNames{idx});
    xlabel('Time [s]');
    legend('Location','best');
    exportgraphics(fig, fullfile(imgDir, sprintf('ekf_state%d_95ci_plotstates.pdf', idx)), 'ContentType','vector');
  end
end

function plotEkfMeasurements(keNom, imgDir, confidence)
% Save 95% confidence plots for the measurements.
  if nargin < 3, confidence = 0.95; end
  measNames = {'z_1 [m]', 'z_2 [m]'};
  ny = size(keNom.y, 1);
  for idx = 1:ny
    fig = figure('Name', sprintf('EKF measurement %d CI', idx), 'Position', [140 140 800 350]);
    hold on; grid on;
    plotmeasurements(keNom, idx, confidence);
    ylabel(measNames{min(idx, numel(measNames))});
    xlabel('Time [s]');
    legend('Location','best');
    exportgraphics(fig, fullfile(imgDir, sprintf('ekf_measurement%d_95ci.pdf', idx)), 'ContentType','vector');
  end
end

function plotLqrTracking(trackingRuns, imgDir)
% Plot tracking errors (world + body frames) and control signals for LQR tuning sweep.
  colors = lines(numel(trackingRuns));

  % World-frame errors
  figW = figure('Name','LQR Errors (world frame)','Position',[100 100 1100 700]);
  errNamesW = {'e_x [m]','e_y [m]','e_\theta [rad]'};
  for idx = 1:3
    subplot(3,1,idx); hold on; grid on;
    for k = 1:numel(trackingRuns)
      plot(trackingRuns(k).t, trackingRuns(k).err_world(:,idx), 'Color', colors(k,:), 'LineWidth',1.4, ...
        'DisplayName', trackingRuns(k).label);
    end
    ylabel(errNamesW{idx});
    if idx == 1, title('Tracking errors (world frame)'); end
    if idx == 3, xlabel('Time [s]'); legend('Location','bestoutside'); end
  end
  exportgraphics(figW, fullfile(imgDir, 'lqr_tracking_errors_world.pdf'), 'ContentType','vector');

  % Body-frame errors (as used by the controller)
  figB = figure('Name','LQR Errors (body frame)','Position',[100 100 1100 700]);
  errNamesB = {'e_{x''} [m]','e_{y''} [m]','e_\theta [rad]'};
  for idx = 1:3
    subplot(3,1,idx); hold on; grid on;
    for k = 1:numel(trackingRuns)
      plot(trackingRuns(k).t, trackingRuns(k).err_body(:,idx), 'Color', colors(k,:), 'LineWidth',1.4, ...
        'DisplayName', trackingRuns(k).label);
    end
    ylabel(errNamesB{idx});
    if idx == 1, title('Tracking errors (body frame, spec 4a)'); end
    if idx == 3, xlabel('Time [s]'); legend('Location','bestoutside'); end
  end
  exportgraphics(figB, fullfile(imgDir, 'lqr_tracking_errors_body.pdf'), 'ContentType','vector');

  % Control signals (feedforward proxy)
  figU = figure('Name','LQR Inputs','Position',[100 100 1100 500]);
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
  exportgraphics(figU, fullfile(imgDir, 'lqr_control_signals.pdf'), 'ContentType','vector');
end
