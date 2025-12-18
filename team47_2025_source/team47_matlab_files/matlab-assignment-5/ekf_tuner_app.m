function ekf_tuner_app
% Simple UI to tune Q, R, and P0 on the offline EKF using ekf_Q1_R1.csv.
% Shows xhat, yhat, thetahat with 95% confidence bands.

  % -----------------------------------------------------------------------
  % Fixed geometry and nominal values (keep in sync with assignment5_solution)
  % -----------------------------------------------------------------------
  Ts     = 0.010;
  alpha  = 0.075;
  beta   = 0.065;
  gamma  = 0.078;
  wall1  = [0, 1, 0];
  wall2  = [1, 0, 0];

  Q_nom  = diag([1e-6, 1e-6, 1e-6]);
  R_nom  = diag([1.2e-3, 1.3e-5]);
  P0_nom = diag([1e-3, 1e-3, 7.6e-3]);
  x0     = [-0.30; -0.20; 0];

  dataFile = fullfile(fileparts(mfilename('fullpath')), 'data', 'ekf_Q1_R1.csv');
  if ~isfile(dataFile)
    error('Cannot find data file: %s', dataFile);
  end
  D = parseQrcAssignment5(dataFile);

  % -----------------------------------------------------------------------
  % UI layout
  % -----------------------------------------------------------------------
  fig = uifigure('Name','EKF Q/R/P0 tuner','Position',[100 100 1150 620]);
  gl = uigridlayout(fig, [1 2]);
  gl.ColumnWidth = {260, '1x'};
  ctrl = uigridlayout(gl, [11 2]);
  ctrl.RowHeight = repmat({30}, 1, 11);
  ctrl.ColumnWidth = {120, 120};

  addLabel(ctrl, 'Q diag [m^2 / rad^2]', 1);
  qxField = addField(ctrl, 'Q_x', Q_nom(1,1), 2);
  qyField = addField(ctrl, 'Q_y', Q_nom(2,2), 3);
  qtField = addField(ctrl, 'Q_\theta', Q_nom(3,3), 4);

  addLabel(ctrl, 'R diag [m^2]', 5);
  r1Field = addField(ctrl, 'R_1', R_nom(1,1), 6);
  r2Field = addField(ctrl, 'R_2', R_nom(2,2), 7);

  addLabel(ctrl, 'P0 diag [m^2 / rad^2]', 8);
  p1Field = addField(ctrl, 'P0_x', P0_nom(1,1), 9);
  p2Field = addField(ctrl, 'P0_y', P0_nom(2,2), 10);
  p3Field = addField(ctrl, 'P0_\theta', P0_nom(3,3), 11);

  runBtn = uibutton(ctrl, 'Text','Run EKF', 'FontWeight','bold', ...
    'ButtonPushedFcn', @(~,~) runEkfAndPlot());
  runBtn.Layout.Row = 11;
  runBtn.Layout.Column = [1 2];

  plots = uigridlayout(gl, [3 1]);
  plots.RowHeight = {'1x','1x','1x'};
  axX = uiaxes(plots); ylabel(axX, '\hat{x} [m]'); grid(axX,'on');
  axY = uiaxes(plots); ylabel(axY, '\hat{y} [m]'); grid(axY,'on');
  axT = uiaxes(plots); ylabel(axT, '\hat{\theta} [rad]'); xlabel(axT,'Time [s]'); grid(axT,'on');

  % -----------------------------------------------------------------------
  % Callback
  % -----------------------------------------------------------------------
  function runEkfAndPlot
    Qk = diag([qxField.Value, qyField.Value, qtField.Value]);
    Rk = diag([r1Field.Value, r2Field.Value]);
    P0 = diag([p1Field.Value, p2Field.Value, p3Field.Value]);

    [xhat, Pdiag] = rerunEkfOffline(D, Ts, alpha, beta, gamma, wall1, wall2, x0, P0, Qk, Rk);
    ci = 1.96 * sqrt(Pdiag); % 95% CI

    plotState(axX, D.time, xhat(1,:), ci(:,1), 'x');
    plotState(axY, D.time, xhat(2,:), ci(:,2), 'y');
    plotState(axT, D.time, xhat(3,:), ci(:,3), '\theta');
  end

  % Initial run
  runEkfAndPlot();
end

% -------------------------------------------------------------------------
% Helpers
% -------------------------------------------------------------------------
function lbl = addLabel(parent, txt, row)
  lbl = uilabel(parent, 'Text', txt, 'FontWeight','bold');
  lbl.Layout.Row = row;
  lbl.Layout.Column = [1 2];
end

function fld = addField(parent, name, val, row)
  lbl = uilabel(parent, 'Text', name, 'HorizontalAlignment','right');
  lbl.Layout.Row = row;
  lbl.Layout.Column = 1;
  fld = uieditfield(parent, 'numeric', 'Value', val, 'LowerLimit', 0);
  fld.Layout.Row = row;
  fld.Layout.Column = 2;
end

function plotState(ax, t, x, ci, name)
  cla(ax);
  hold(ax, 'on');
  up = x + ci.';
  lo = x - ci.';
  fill(ax, [t; flipud(t)], [up'; flipud(lo')], [0.9 0.9 1], 'EdgeColor','none', 'FaceAlpha',0.4);
  plot(ax, t, x, 'b', 'LineWidth',1.4);
  title(ax, sprintf('%s with 95%% CI', name));
  grid(ax, 'on');
end

function [xhat, Pdiag] = rerunEkfOffline(D, Ts, alpha, beta, gamma, w1, w2, x0, P0, Qk, Rk)
  N = numel(D.time);
  xhat = zeros(3, N);
  Pdiag = zeros(N,3);
  x = x0(:);
  P = P0;

  for k = 1:N
    if k > 1
      u = [D.v(k-1); D.omega(k-1)];
      [x, P] = ekfPredictEuler(x, P, u, Ts, Qk);
    end

    y = [D.z1(k); D.z2(k)];
    idx = isfinite(y);
    if any(idx)
      y = y(idx);
      [zPred, C] = measurementModel(x, alpha, beta, gamma, w1, w2);
      zPred = zPred(idx);
      C = C(idx,:);
      Ruse = Rk(idx, idx);
      S = C * P * C' + Ruse;
      K = P * C' / S;
      x = x + K * (y - zPred);
      P = P - K * C * P;
    end

    x(3) = wrapToPiLocal(x(3));
    xhat(:,k) = x;
    Pdiag(k,:) = diag(P);
  end
end

function [x, P] = ekfPredictEuler(x, P, u, Ts, Qk)
  v = u(1);
  omega = u(2);
  th = x(3);
  cth = cos(th);
  sth = sin(th);
  x = x + Ts * [v * cth; v * sth; omega];
  A = [1, 0, -Ts * v * sth;
       0, 1,  Ts * v * cth;
       0, 0,  1];
  P = A * P * A' + Qk;
end

function [z, C] = measurementModel(x, alpha, beta, gamma, w1, w2)
  theta = x(3);
  cth = cos(theta);
  sth = sin(theta);
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

function th = wrapToPiLocal(th)
  th = mod(th + pi, 2*pi) - pi;
end

function D = parseQrcAssignment5(filename)
  opts = detectImportOptions(filename);
  opts.DataLines = [3 inf];
  opts.VariableNamesLine = 2;
  T = readtable(filename, opts);
  N = height(T);
  tRaw = T.Time(:);
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
  D.v_ff      = pickVar(T, {'ValueIn0','v_ff'}, zeros(N,1));
  D.omega_ff  = pickVar(T, {'ValueIn1','omega_ff'}, zeros(N,1));
  D.z1        = pickVar(T, {'ValueIn2','z1','y1'}, NaN(N,1));
  D.z2        = pickVar(T, {'ValueIn3','z2','y2'}, NaN(N,1));
  D.xhat      = pickVar(T, {'ValueIn4','xhat','x1'}, zeros(N,1));
  D.yhat      = pickVar(T, {'ValueIn5','yhat','x2'}, zeros(N,1));
  D.thetahat  = pickVar(T, {'ValueIn6','thetahat','x3'}, zeros(N,1));
  D.Pxx       = pickVar(T, {'ValueIn7','Pxx','P11'}, 1e-6 * ones(N,1));
  D.Pyy       = pickVar(T, {'ValueIn8','Pyy','P22'}, 1e-6 * ones(N,1));
  D.Ptt       = pickVar(T, {'ValueIn9','Ptt','P33'}, 1e-6 * ones(N,1));
  D.v         = pickVar(T, {'ValueIn10','v','v_applied'}, D.v_ff);
  D.omega     = pickVar(T, {'ValueIn11','omega','omega_applied'}, D.omega_ff);
end

function out = pickVar(T, names, default)
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
    out = out * ones(height(T),1);
  elseif isvector(out)
    out = out(:);
  end
end
