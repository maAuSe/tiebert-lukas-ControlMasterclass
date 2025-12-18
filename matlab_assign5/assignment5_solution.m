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

% Walls p*x + q*y = r. Corner at (0,0), cart starts in x<0, y<0 facing +X.
% W1: front sensor measures to wall x=0 (AHEAD of robot)
% W2: side sensor measures to wall y=0 (to the LEFT of robot)
wall1 = [1, 0, 0];  % x = 0 (front wall W1)
wall2 = [0, 1, 0];  % y = 0 (side wall W2)
geom  = struct('alpha',alpha,'beta',beta,'gamma',gamma,'wall1',wall1,'wall2',wall2);

%% ========================================================================
%  NOISE COVARIANCES & INITIAL STATE (PLACEHOLDERS TO TUNE)
%  ========================================================================
% Nominal EKF covariances (must match Arduino firmware: arduino_files/CT-EKF-Swivel/extended_kalman_filter.cpp)
% Tuned values from assignment5.tex eq. (14)-(15)
Q_proc_base = diag([8e-9, 9e-8, 9e-7]);   % process noise (m^2, m^2, rad^2)
R_meas_base = diag([0.0198, 0.09]);       % measurement noise (m^2)

firmware_Q_scale = 1.0;      % keep unity so MATLAB/Arduino match exactly
firmware_R_scale = 1.0;

Q_proc_nom = Q_proc_base * firmware_Q_scale;
R_meas_nom = R_meas_base * firmware_R_scale;
P0_default = diag([1e-3, 1e-3, 7.6e-3]); % must match resetKalmanFilter() on Arduino
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

Q_lqr_A = diag([16, 16, 4]);
R_lqr_A = diag([0.1, 0.1]);

lqrRuns = [ ...
  struct('label','LQR-A','Q',Q_lqr_A,          'R',R_lqr_A,            'file',fullfile(dataDir,'lqr_A.csv')); ...
  struct('label','LQR-B','Q',Q_lqr_A*2,        'R',R_lqr_A,            'file',fullfile(dataDir,'lqr_B.csv')); ...
  struct('label','LQR-C','Q',Q_lqr_A,          'R',R_lqr_A*2,          'file',fullfile(dataDir,'lqr_C.csv')); ...
  struct('label','LQR-D','Q',Q_lqr_A*4,        'R',R_lqr_A*0.5,        'file',fullfile(dataDir,'lqr_D.csv'))  ...
];

lqrActiveIdx = 4;
Q_lqr = lqrRuns(lqrActiveIdx).Q;
R_lqr = lqrRuns(lqrActiveIdx).R;
K_dlqr = dlqr(Ad, Bd, Q_lqr, R_lqr);

K_firmware = -K_dlqr;
fprintf('Active LQR run: %s\n', lqrRuns(lqrActiveIdx).label);
fprintf('K_firmware (copy to Arduino):\n');
fprintf('  _Klqr = {%.4ff, %.4ff, %.4ff,\n', K_firmware(1,1), K_firmware(1,2), K_firmware(1,3));
fprintf('           %.4ff, %.4ff, %.4ff};\n', K_firmware(2,1), K_firmware(2,2), K_firmware(2,3));

K_lqr = K_dlqr;

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
%  OPTIONAL: OFFLINE EKF RE-RUN (Q SWEEP TO PICK BEST)
%  ========================================================================
% Re-run the EKF in MATLAB using the logged inputs/measurements and a wide
% sweep of Q scaling values. The "best" run is the one with the smallest
% summed whitened innovation + log(|S|) score (approx. negative log-lik.).
doOfflineEkfSweep = true;
offlineEkfFile = fullfile(dataDir, 'ekf_Q1_R1.csv');
Q_sweep_scales = [0.01, 0.05, 0.1, 0.25, 0.5, 1, 2, 4, 8]; % wide range

if doOfflineEkfSweep && isfile(offlineEkfFile)
  Doff = parseQrcAssignment5(offlineEkfFile);
  colors = lines(numel(Q_sweep_scales));
  offlineRuns = struct([]);

  for k = 1:numel(Q_sweep_scales)
    Qk = Q_proc_nom * Q_sweep_scales(k);
    [xhatOff, diagOff] = rerunEkfOffline(Doff, Ts, geom, x0_default, P0_default, Qk, R_meas_nom);
    offlineRuns(k).scale = Q_sweep_scales(k); %#ok<SAGROW>
    offlineRuns(k).xhat  = xhatOff;
    offlineRuns(k).score = diagOff.negLogLik;
    offlineRuns(k).scoreInnov = diagOff.whitenedInnovSum;
  end

  [~, bestIdx] = min([offlineRuns.score]);

  figure('Name', 'Offline EKF (Q sweep): x,y,\theta', 'Position', [140 160 920 460]);
  tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');
  stateNames = {'\hat{x} [m]', '\hat{y} [m]', '\hat{\theta} [rad]'};
  for sIdx = 1:3
    nexttile; hold on; grid on;
    for k = 1:numel(offlineRuns)
      lw = 0.8 + 0.8 * (k == bestIdx);
      plot(Doff.time, offlineRuns(k).xhat(sIdx,:), 'Color', colors(k,:), ...
        'LineWidth', lw, 'DisplayName', sprintf('Q x%.2g', offlineRuns(k).scale));
    end
    ylabel(stateNames{sIdx});
    if sIdx == 1
      title(sprintf('Offline EKF Q sweep (best: Q x%.2g)', offlineRuns(bestIdx).scale));
    end
    if sIdx == 3
      xlabel('Time [s]');
      legend('Location','eastoutside');
    end
  end

  figure('Name', 'Offline EKF Q sweep score', 'Position', [180 180 720 320]);
  hold on; grid on;
  bar(Q_sweep_scales, [offlineRuns.score]);
  set(gca, 'XScale','log');
  xticks(Q_sweep_scales);
  xticklabels(compose('%.2g', Q_sweep_scales));
  xlabel('Q scaling factor (diag)');
  ylabel('0.5 \cdot \Sigma ( \nu^T S^{-1} \nu + \log|S| )');
  title('Offline EKF Q sweep score (lower is better)');
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
  D = parseLqrCsv(lqrRuns(k).file);
  trackingRuns(end+1).label = lqrRuns(k).label; %#ok<SAGROW>
  trackingRuns(end).t         = D.time;
  trackingRuns(end).err_world = [D.x_ref - D.xhat, D.y_ref - D.yhat, wrapToPiLocal(D.theta_ref - D.thetahat)];
  trackingRuns(end).err_body  = [D.ex_body, D.ey_body, D.etheta_body];
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

  % Reconstruct reference from feedforward for EKF experiments
  x0 = -0.30;
  y0 = -0.20;
  theta0 = 0.0;
  [D.x_ref, D.y_ref, D.theta_ref] = reconstructReferenceFromVOmega(D.time, D.v_ff, D.omega_ff, x0, y0, theta0);
end

function D = parseLqrCsv(filename)
% Parse LQR experiment CSV (channels 0-11 from firmware)
  opts = detectImportOptions(filename);
  opts.DataLines = [3 inf];
  opts.VariableNamesLine = 2;
  T = readtable(filename, opts);
  startRow = lqrButton2StartRow(filename); % align on button-2 (trajectory start)
  if ~isempty(startRow)
    startRow = min(startRow, height(T));
    T = T(startRow:end, :);
  end
  N = height(T);
  getv = @(names, default) pickVar(T, names, default, N);

  tRaw = T.Time;
  tRaw = tRaw(:);
  tRel = tRaw - tRaw(1);
  dtMed = median(diff(tRel), 'omitnan');
  if ~isfinite(dtMed), dtMed = NaN; end
  if ~isnan(dtMed) && dtMed > 500
    D.time = tRel / 1e6;
  elseif ~isnan(dtMed) && dtMed > 0.5
    D.time = tRel / 1000;
  else
    D.time = tRel;
  end
  D.time = D.time(:);

  % LQR channel mapping (0-11)
  D.x_ref       = getv({'ValueIn0','x_ref'}, zeros(N,1));
  D.y_ref       = getv({'ValueIn1','y_ref'}, zeros(N,1));
  D.theta_ref   = getv({'ValueIn2','theta_ref'}, zeros(N,1));
  D.xhat        = getv({'ValueIn3','xhat'}, zeros(N,1));
  D.yhat        = getv({'ValueIn4','yhat'}, zeros(N,1));
  D.thetahat    = getv({'ValueIn5','thetahat'}, zeros(N,1));
  D.ex_body     = getv({'ValueIn6','ex_body'}, zeros(N,1));
  D.ey_body     = getv({'ValueIn7','ey_body'}, zeros(N,1));
  D.etheta_body = getv({'ValueIn8','etheta_body'}, zeros(N,1));
  D.v           = getv({'ValueIn9','v_applied'}, zeros(N,1));
  D.omega       = getv({'ValueIn10','omega_applied'}, zeros(N,1));
  D.z1          = getv({'ValueIn11','z1'}, NaN(N,1));
end

function startRow = lqrButton2StartRow(filename)
% Manual offsets (1-based table rows) where button 2 was pressed per log.
  [~, name, ~] = fileparts(filename);
  switch lower(name)
    case 'lqr_a',    startRow = 162;
    case 'lqr_b',    startRow = 219;
    case 'lqr_c',    startRow = 137;
    case 'lqr_d',    startRow = 159;
    case 'test_lqr', startRow = 825;
    otherwise,       startRow = [];
  end
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

function [xhat, diagOut] = rerunEkfOffline(D, Ts, geom, x0, P0, Qk, Rk)
% Re-run the same EKF structure as the Arduino firmware, using logged u/y.
% Returns xhat as 3xN (x,y,theta).

  N = numel(D.time);
  xhat = zeros(3, N);
  x = x0(:);
  P = P0;
  innov = NaN(2, N);
  innovCost = NaN(1, N);
  logdetS = NaN(1, N);

  for k = 1:N
    if k > 1
      u = [D.v(k-1); D.omega(k-1)];
      [x, P] = ekfPredictEuler(x, P, u, Ts, Qk);
    end

    if isfinite(D.z1(k)) && isfinite(D.z2(k))
      y = [D.z1(k); D.z2(k)];
      [x, P, nu, S] = ekfCorrectWallRanges(x, P, y, geom, Rk);
      innov(:,k) = nu;
      innovCost(k) = nu' * (S \ nu);
      try
        L = chol(S, 'lower');
        logdetS(k) = 2 * sum(log(diag(L)));
      catch
        logdetS(k) = log(det(S));
      end
    end

    x(3) = wrapToPiLocal(x(3));
    xhat(:,k) = x;
  end

  measMask = isfinite(innovCost);
  diagOut = struct();
  diagOut.innov = innov;
  diagOut.innovRms = sqrt(nanmean(innov(:).^2));
  diagOut.whitenedInnovSum = nansum(innovCost);
  if any(measMask)
    diagOut.negLogLik = 0.5 * nansum(innovCost(measMask) + logdetS(measMask));
  else
    diagOut.negLogLik = Inf;
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

function [x, P, nu, S] = ekfCorrectWallRanges(x, P, y, geom, Rk)
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

function plotEkfStateSweep(ekfExps, imgDir, ~)
% Per-state overlay of EKF estimates for different Q/R choices.
% Color-agnostic: uses distinct line styles and markers for B/W printing.
% No error bands are drawn for comparison clarity.
  lineStyles = {'-', '--', ':', '-.'};
  markers    = {'none', 'o', 's', 'd'};
  markerStep = 80;  % plot a marker every N samples to avoid clutter
  stateNames = {'x_c [m]', 'y_c [m]', '\theta [rad]'};
  nExp = numel(ekfExps);
  for idx = 1:3
    fig = figure('Name', sprintf('EKF state %d sweep', idx), 'Position', [100 100 850 500]);
    hold on; grid on;
    for k = 1:nExp
      ke = ekfExps(k).ke;
      t  = ke.t(:);
      xk = ke.x(idx, :);
      styleIdx = mod(k-1, numel(lineStyles)) + 1;
      markIdx  = mod(k-1, numel(markers)) + 1;
      plot(t, xk, 'LineStyle', lineStyles{styleIdx}, 'Color', 'k', ...
           'Marker', markers{markIdx}, 'MarkerIndices', 1:markerStep:numel(t), ...
           'MarkerSize', 4, 'LineWidth', 1.2, 'DisplayName', ekfExps(k).label);
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
% Save 95% confidence plots for measurements (color-agnostic).
% Uses solid line for measurement, dashed for prediction, and dotted for bounds.
% Confidence bounds based on R (measurement noise), not S (innovation cov).
  if nargin < 3, confidence = 0.95; end
  measNames = {'Front sensor z_1 [m]', 'Side sensor z_2 [m]'};
  ny = size(keNom.y, 1);
  z_alpha = norminv(1 - (1-confidence)/2);
  for idx = 1:ny
    fig = figure('Name', sprintf('EKF measurement %d CI', idx), 'Position', [140 140 800 350]);
    hold on; grid on;
    t = keNom.t(:);
    yMeas = keNom.y(idx, :);
    % Predicted measurement = y - innovation
    yPred = yMeas(:) - keNom.nu(idx, :)';
    % Use R (measurement noise) for confidence bounds, clamped to avoid inflated values
    Rdiag = squeeze(keNom.R(idx, idx, :));
    Rdiag = min(Rdiag, 1);  % clamp inflated R values (sensor-off periods)
    sigma = sqrt(Rdiag(:));
    upper = yPred + z_alpha * sigma;
    lower = yPred - z_alpha * sigma;
    % Only plot where measurements are valid (not NaN)
    validMask = isfinite(yMeas);
    tValid = t(validMask);
    yMeasValid = yMeas(validMask);
    yPredValid = yPred(validMask);
    upperValid = upper(validMask);
    lowerValid = lower(validMask);
    % Plot bounds as dotted, prediction as dashed, measurement as solid
    plot(tValid, upperValid, ':k', 'LineWidth', 1.0, 'HandleVisibility','off');
    plot(tValid, lowerValid, ':k', 'LineWidth', 1.0, 'DisplayName', sprintf('%.0f%% bounds', confidence*100));
    plot(tValid, yPredValid, '--k', 'LineWidth', 1.2, 'DisplayName', 'Predicted');
    plot(tValid, yMeasValid, '-k', 'LineWidth', 1.4, 'DisplayName', 'Measured');
    ylabel(measNames{min(idx, numel(measNames))});
    xlabel('Time [s]');
    legend('Location','best');
    exportgraphics(fig, fullfile(imgDir, sprintf('ekf_measurement%d_95ci.pdf', idx)), 'ContentType','vector');
  end
end

function plotLqrTracking(trackingRuns, imgDir)
% Plot tracking errors (world + body frames) and control signals for LQR tuning sweep.
% Black & white compatible: uses distinct line styles and markers for print clarity.
% Square aspect ratio figures for side-by-side LaTeX layout.
  lineStyles = {'-', '--', ':', '-.'};
  markers    = {'none', 'o', 's', '^'};
  markerStep = 100;  % sparse markers to avoid clutter
  nRuns = numel(trackingRuns);

  % World-frame errors (square figure)
  figW = figure('Name','LQR Errors (world frame)','Position',[100 100 520 560]);
  tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
  errNamesW = {'e_x [m]','e_y [m]','e_\theta [rad]'};
  for idx = 1:3
    nexttile; hold on; grid on;
    for k = 1:nRuns
      styleIdx = mod(k-1, numel(lineStyles)) + 1;
      markIdx  = mod(k-1, numel(markers)) + 1;
      t = trackingRuns(k).t;
      plot(t, trackingRuns(k).err_world(:,idx), ...
           'LineStyle', lineStyles{styleIdx}, 'Color', 'k', ...
           'Marker', markers{markIdx}, 'MarkerIndices', 1:markerStep:numel(t), ...
           'MarkerSize', 3, 'LineWidth', 1.1, 'DisplayName', trackingRuns(k).label);
    end
    ylabel(errNamesW{idx}, 'FontSize', 9);
    set(gca, 'FontSize', 8);
    if idx == 3
      xlabel('Time [s]', 'FontSize', 9);
      lgd = legend('Location','southeast','FontSize',7);
      lgd.ItemTokenSize = [12,8];
    end
  end
  exportgraphics(figW, fullfile(imgDir, 'lqr_tracking_errors_world.pdf'), 'ContentType','vector');

  % Body-frame errors (square figure)
  figB = figure('Name','LQR Errors (body frame)','Position',[100 100 520 560]);
  tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
  errNamesB = {'e_{x''} [m]','e_{y''} [m]','e_\theta [rad]'};
  for idx = 1:3
    nexttile; hold on; grid on;
    for k = 1:nRuns
      styleIdx = mod(k-1, numel(lineStyles)) + 1;
      markIdx  = mod(k-1, numel(markers)) + 1;
      t = trackingRuns(k).t;
      plot(t, trackingRuns(k).err_body(:,idx), ...
           'LineStyle', lineStyles{styleIdx}, 'Color', 'k', ...
           'Marker', markers{markIdx}, 'MarkerIndices', 1:markerStep:numel(t), ...
           'MarkerSize', 3, 'LineWidth', 1.1, 'DisplayName', trackingRuns(k).label);
    end
    ylabel(errNamesB{idx}, 'FontSize', 9);
    set(gca, 'FontSize', 8);
    if idx == 3
      xlabel('Time [s]', 'FontSize', 9);
      lgd = legend('Location','southeast','FontSize',7);
      lgd.ItemTokenSize = [12,8];
    end
  end
  exportgraphics(figB, fullfile(imgDir, 'lqr_tracking_errors_body.pdf'), 'ContentType','vector');

  % Control signals (compact figure)
  figU = figure('Name','LQR Inputs','Position',[100 100 600 380]);
  tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
  nexttile; hold on; grid on;
  for k = 1:nRuns
    styleIdx = mod(k-1, numel(lineStyles)) + 1;
    markIdx  = mod(k-1, numel(markers)) + 1;
    t = trackingRuns(k).t;
    plot(t, trackingRuns(k).u(:,1), ...
         'LineStyle', lineStyles{styleIdx}, 'Color', 'k', ...
         'Marker', markers{markIdx}, 'MarkerIndices', 1:markerStep:numel(t), ...
         'MarkerSize', 3, 'LineWidth', 1.1, 'DisplayName', trackingRuns(k).label);
  end
  ylabel('v [m/s]', 'FontSize', 9);
  set(gca, 'FontSize', 8);
  lgd = legend('Location','northeast','FontSize',7);
  lgd.ItemTokenSize = [12,8];
  nexttile; hold on; grid on;
  for k = 1:nRuns
    styleIdx = mod(k-1, numel(lineStyles)) + 1;
    markIdx  = mod(k-1, numel(markers)) + 1;
    t = trackingRuns(k).t;
    plot(t, trackingRuns(k).u(:,2), ...
         'LineStyle', lineStyles{styleIdx}, 'Color', 'k', ...
         'Marker', markers{markIdx}, 'MarkerIndices', 1:markerStep:numel(t), ...
         'MarkerSize', 3, 'LineWidth', 1.1, 'DisplayName', trackingRuns(k).label);
  end
  ylabel('\omega [rad/s]', 'FontSize', 9);
  xlabel('Time [s]', 'FontSize', 9);
  set(gca, 'FontSize', 8);
  exportgraphics(figU, fullfile(imgDir, 'lqr_control_signals.pdf'), 'ContentType','vector');
end
