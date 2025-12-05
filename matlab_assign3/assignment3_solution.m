%% Assignment 3 - State feedback and state estimation
% Pole-placement design, simulation, and experimental data plotting.
% Data files use QRC format with columns:
%   Time, ValueIn0 (ref), ValueIn1 (distance), ValueIn2 (xhat),
%   ValueIn3 (v_ref), ValueIn4 (speedA), ValueIn5 (speedB),
%   ValueIn6 (voltA), ValueIn7 (voltB), ValueIn8 (nu), ValueIn9 (xref)

clear; close all; clc;

%% ========================================================================
%  CONFIGURATION
%  ========================================================================
Ts = 0.01;                    % s (scheduler period)
r = 0.033;                    % m (wheel radius)
Ad = 1;                       % discrete-time state matrix (scalar)
Bd = Ts * r;                  % discrete-time input matrix (m/rad)
Cd = -1; Dd = 0;              % measurement: y = -x (distance is positive)

% Gain sweeps for analysis
K_sweep = [20, 40, 80];           % rad/(s*m) - controller gains
L_sweep = [-0.05, -0.18, -0.35];  % observer gains for 2(a)
K_nom = 40;                       % nominal controller gain
L_nom = -0.18;                    % nominal estimator gain
p_cl_nom = 1 - Bd * K_nom;        % closed-loop pole with K_nom
L_slow = -(1 - p_cl_nom) / 10;    % 10x slower estimator pole for 2(c): p_est = 1 + L_slow
p_est_slow = 1 + L_slow;

% Simulation parameters
Tsim = 4;                       % s
t_sim = 0:Ts:Tsim;
x_true0 = -0.30;                % m (start 30 cm from wall)
x_ref = -0.15;                  % m (target 15 cm from wall)
xhat_init_wrong = -0.40;        % m (wrong initial estimate)

% Data files (estimator-only experiments available)
dataDir = 'C:\\Users\\campa\\Documents\\Arduino\\matlab_assign3\\data';
estOnlyFiles = {
    fullfile(dataDir, 'est_only_L-005.csv'), -0.05, 188, 388;   % file, L, startRow, endRow
    fullfile(dataDir, 'est_only_L-018.csv'), -0.18, 130, 330;
    fullfile(dataDir, 'est_only_L-035.csv'), -0.35, 200, 400;
};

% Controller-only files (to be recorded)
% Format: {filename, K, startRow, endRow}
ctrlOnlyFiles = {
    fullfile(dataDir, 'ctrl_only_K20.csv'),  20, 181, 581;
    fullfile(dataDir, 'ctrl_only_K40.csv'),  40, 235, 635;
    fullfile(dataDir, 'ctrl_only_K80.csv'),  80, 154, 554;
};

% Estimator + Controller files (to be recorded)
% Format: {filename, label, startRow, endRow}
estCtrlFiles = {
    fullfile(dataDir, 'est_ctrl_good_init.csv'), 'Good initial estimate', 1450, 2250;
    fullfile(dataDir, 'est_ctrl_bad_init.csv'),  'Wrong initial estimate', 203, 1003;
};

%% ========================================================================
%  1(c) CLOSED-LOOP POLE MAP VS K
%  ========================================================================
% Closed-loop pole: p_cl = A - B*K = 1 - Ts*r*K
p_cl = 1 - Bd * K_sweep;

figure('Name','Pole Map vs K','Position',[100 100 600 500]);
hold on; grid on;
theta = linspace(0, 2*pi, 100);
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 0.5, 'HandleVisibility','off'); % unit circle
scatter(real(p_cl), imag(p_cl), 100, 'x', 'LineWidth', 2);
xline(0, ':k', 'HandleVisibility','off');
yline(0, ':k', 'HandleVisibility','off');
for i = 1:length(K_sweep)
    text(real(p_cl(i))+0.02, imag(p_cl(i))+0.03, sprintf('K = %d', K_sweep(i)), 'FontSize', 10);
end
xlabel('Real Axis'); ylabel('Imaginary Axis');
title('Closed-loop pole location vs K');
xlim([-0.2 1.2]); ylim([-0.6 0.6]);
axis equal;

% Additional: show pole trajectory for continuous K
K_range = linspace(0, 350, 100);
p_cl_range = 1 - Bd * K_range;
plot(p_cl_range, zeros(size(p_cl_range)), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Pole trajectory');
legend('Poles for K sweep', 'Pole trajectory', 'Location', 'northwest');

%% ========================================================================
%  1(c) SIMULATED STEP RESPONSES FOR K SWEEP
%  ========================================================================
figure('Name','Simulated Step Response vs K','Position',[100 100 700 450]);
hold on; grid on;
% Black-white friendly: use line styles and markers
lineStyles = {'-', '--', ':'};
markers = {'o', 's', '^'};
markerInterval = 40;  % plot marker every N points

for i = 1:length(K_sweep)
    kVal = K_sweep(i);
    xk = zeros(size(t_sim));
    xk(1) = x_true0;
    for k = 1:(numel(t_sim)-1)
        % Closed-loop: x[k+1] = (1 - B*K)*x[k] + B*K*x_ref
        xk(k+1) = (1 - Bd * kVal) * xk(k) + Bd * kVal * x_ref;
    end
    % Plot distance (= -x) since IR sensor measures positive distance
    plot(t_sim, -xk, 'k', 'LineStyle', lineStyles{i}, 'LineWidth', 1.5, ...
         'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t_sim), ...
         'MarkerSize', 6, 'DisplayName', sprintf('K = %d (p = %.3f)', kVal, 1 - Bd*kVal));
end
plot(t_sim, -x_ref*ones(size(t_sim)), 'k-.', 'LineWidth', 1.5, 'DisplayName', 'Reference');
xlabel('Time [s]'); ylabel('Distance to wall [m]');
title('Simulated step response vs K (ideal velocity loop)');
legend('Location','best');

%% ========================================================================
%  1(d) ESTIMATOR POLE MAP VS L
%  ========================================================================
% Estimator pole: p_est = A - L*C = 1 - L*(-1) = 1 + L
p_est = 1 + L_sweep;

figure('Name','Estimator Pole Map vs L','Position',[100 100 600 500]);
hold on; grid on;
plot(cos(theta), sin(theta), 'k--', 'LineWidth', 0.5, 'HandleVisibility','off');
scatter(real(p_est), imag(p_est), 100, 'o', 'LineWidth', 2, 'MarkerFaceColor', 'flat');
xline(0, ':k', 'HandleVisibility','off');
yline(0, ':k', 'HandleVisibility','off');
for i = 1:length(L_sweep)
    text(real(p_est(i))+0.02, imag(p_est(i))+0.03, sprintf('L = %.2f', L_sweep(i)), 'FontSize', 10);
end
xlabel('Real Axis'); ylabel('Imaginary Axis');
title('Estimator pole location vs L');
xlim([-0.2 1.2]); ylim([-0.6 0.6]);
axis equal;

% Show pole trajectory
L_range = linspace(0, -1, 100);
p_est_range = 1 + L_range;
plot(p_est_range, zeros(size(p_est_range)), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Pole trajectory');
legend('Poles for L sweep', 'Pole trajectory', 'Location', 'northwest');

%% ========================================================================
%  1(d) SIMULATED ESTIMATOR CONVERGENCE VS L
%  ========================================================================
figure('Name','Simulated Estimator Convergence','Position',[100 100 700 450]);
hold on; grid on;
% Black-white friendly: use line styles and markers
lineStyles = {'-', '--', ':'};
markers = {'o', 's', '^'};
markerInterval = 40;

for i = 1:length(L_sweep)
    Lval = L_sweep(i);
    xhat = zeros(size(t_sim));
    xhat(1) = xhat_init_wrong;
    x_true = x_true0;  % cart stationary
    
    for k = 1:(numel(t_sim)-1)
        % Prediction: xhat_pred = A*xhat + B*u = xhat (u=0, A=1)
        xhat_pred = xhat(k);
        % Measurement: y = C*x = -x_true (positive distance)
        y = -x_true;
        % Innovation: nu = y - C*xhat_pred = y - (-xhat_pred) = y + xhat_pred
        nu = y - (Cd * xhat_pred);
        % Correction: xhat = xhat_pred + L*nu
        xhat(k+1) = xhat_pred + Lval * nu;
    end
    % Plot estimated distance (= -xhat)
    plot(t_sim, -xhat, 'k', 'LineStyle', lineStyles{i}, 'LineWidth', 1.5, ...
         'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t_sim), ...
         'MarkerSize', 6, 'DisplayName', sprintf('L = %.2f (p = %.2f)', Lval, 1 + Lval));
end
plot(t_sim, -x_true0*ones(size(t_sim)), 'k-.', 'LineWidth', 2, 'DisplayName', 'True distance');
xlabel('Time [s]'); ylabel('Estimated distance [m]');
title('Simulated estimator convergence vs L (cart stationary)');
legend('Location','best');

%% ========================================================================
%  2(a) EXPERIMENTAL: ESTIMATOR-ONLY CONVERGENCE
%  ========================================================================
if ~isempty(estOnlyFiles)
    figure('Name','Experimental Estimator Convergence','Position',[100 100 800 500]);
    hold on; grid on;
    % Black-white friendly: grayscale + line styles + markers
    grayLevels = [0.0, 0.4, 0.7];  % dark to light gray
    lineStyles = {'-', '--', ':'};
    markers = {'o', 's', '^'};
    markerInterval = 20;
    
    for i = 1:size(estOnlyFiles, 1)
        filename = estOnlyFiles{i, 1};
        Lval = estOnlyFiles{i, 2};
        startRow = estOnlyFiles{i, 3};
        endRow   = estOnlyFiles{i, 4};
        
        if exist(filename, 'file')
            Tall = readQRCData(filename);
            n = height(Tall);
            i1 = max(1, startRow);
            i2 = min(n, endRow);
            T = Tall(i1:i2, :);
            
            % Normalize time to start at 0
            t = (T.Time - T.Time(1)) / 1000;  % convert ms to s
            grayColor = grayLevels(i) * [1 1 1];
            
            % Plot measured distance (thin, no marker) and estimated distance (thick, with marker)
            plot(t, T.distance, 'Color', grayColor, 'LineStyle', ':', 'LineWidth', 1, ...
                 'DisplayName', sprintf('Measured (L=%.2f)', Lval));
            plot(t, -T.xhat, 'Color', grayColor, 'LineStyle', lineStyles{i}, 'LineWidth', 1.8, ...
                 'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t), ...
                 'MarkerSize', 5, 'MarkerFaceColor', grayColor, ...
                 'DisplayName', sprintf('Estimated (L=%.2f)', Lval));
        else
            warning('File not found: %s', filename);
        end
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Experimental estimator convergence for different L values');
    legend('Location','best');
    xlim([0 2]);
end

%% ========================================================================
%  2(a) EXPERIMENTAL: INNOVATION SIGNAL
%  ========================================================================
if ~isempty(estOnlyFiles)
    figure('Name','Innovation Signal','Position',[100 100 800 400]);
    hold on; grid on;
    % Black-white friendly
    grayLevels = [0.0, 0.4, 0.7];
    lineStyles = {'-', '--', ':'};
    markers = {'o', 's', '^'};
    markerInterval = 20;
    
    for i = 1:size(estOnlyFiles, 1)
        filename = estOnlyFiles{i, 1};
        Lval = estOnlyFiles{i, 2};
        startRow = estOnlyFiles{i, 3};
        endRow   = estOnlyFiles{i, 4};
        
        if exist(filename, 'file')
            Tall = readQRCData(filename);
            n = height(Tall);
            i1 = max(1, startRow);
            i2 = min(n, endRow);
            T = Tall(i1:i2, :);
            t = (T.Time - T.Time(1)) / 1000;
            grayColor = grayLevels(i) * [1 1 1];
            
            plot(t, T.nu, 'Color', grayColor, 'LineStyle', lineStyles{i}, 'LineWidth', 1.5, ...
                 'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t), ...
                 'MarkerSize', 5, 'MarkerFaceColor', grayColor, ...
                 'DisplayName', sprintf('L = %.2f', Lval));
        end
    end
    xlabel('Time [s]'); ylabel('Innovation \nu [m]');
    title('Innovation signal for different L values');
    legend('Location','best');
    xlim([0 2]);
end

%% ========================================================================
%  2(b) EXPERIMENTAL: CONTROLLER-ONLY STEP RESPONSES
%  ========================================================================
if ~isempty(ctrlOnlyFiles)
    % Position response plot
    figure('Name','Controller-Only Position Response','Position',[100 100 800 500]);
    hold on; grid on;
    % Black-white friendly
    grayLevels = [0.0, 0.35, 0.6];
    lineStyles = {'-', '--', ':'};
    markers = {'o', 's', '^'};
    markerInterval = 40;
    
    for i = 1:size(ctrlOnlyFiles, 1)
        filename = ctrlOnlyFiles{i, 1};
        Kval = ctrlOnlyFiles{i, 2};
        startRow = ctrlOnlyFiles{i, 3};
        endRow   = ctrlOnlyFiles{i, 4};
        
        if exist(filename, 'file')
            Tall = readQRCData(filename);
            n = height(Tall);
            i1 = max(1, startRow);
            i2 = min(n, endRow);
            T = Tall(i1:i2, :);
            t = (T.Time - T.Time(1)) / 1000;
            grayColor = grayLevels(i) * [1 1 1];
            
            plot(t, T.distance, 'Color', grayColor, 'LineStyle', lineStyles{i}, 'LineWidth', 1.8, ...
                 'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t), ...
                 'MarkerSize', 5, 'MarkerFaceColor', grayColor, ...
                 'DisplayName', sprintf('K = %d', Kval));
            % Plot reference if available
            if i == 1
                refLine = 0.25 * ones(size(t));
                plot(t, refLine, 'k-.', 'LineWidth', 2, ...
                     'DisplayName', 'Reference (0.25 m)');
            end
        else
            warning('File not found: %s', filename);
        end
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Controller-only step response for different K values (ref = 0.25 m)');
    legend('Location','best');
    
    % Control signal (voltage) plot
    figure('Name','Controller-Only Voltage','Position',[100 100 800 400]);
    hold on; grid on;
    
    for i = 1:size(ctrlOnlyFiles, 1)
        filename = ctrlOnlyFiles{i, 1};
        Kval = ctrlOnlyFiles{i, 2};
        startRow = ctrlOnlyFiles{i, 3};
        endRow   = ctrlOnlyFiles{i, 4};
        
        if exist(filename, 'file')
            Tall = readQRCData(filename);
            n = height(Tall);
            i1 = max(1, startRow);
            i2 = min(n, endRow);
            T = Tall(i1:i2, :);
            t = (T.Time - T.Time(1)) / 1000;
            grayColor = grayLevels(i) * [1 1 1];
            
            % Average voltage of both motors
            volt_avg = 0.5 * (T.voltA + T.voltB);
            plot(t, volt_avg, 'Color', grayColor, 'LineStyle', lineStyles{i}, 'LineWidth', 1.5, ...
                 'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t), ...
                 'MarkerSize', 5, 'MarkerFaceColor', grayColor, ...
                 'DisplayName', sprintf('K = %d', Kval));
        end
    end
    xlabel('Time [s]'); ylabel('Average voltage [V]');
    title('Control signal for different K values');
    legend('Location','best');
end

%% ========================================================================
%  2(c) EXPERIMENTAL: ESTIMATOR + CONTROLLER
%  ========================================================================
if ~isempty(estCtrlFiles)
    figure('Name','Estimator + Controller','Position',[100 100 800 500]);
    hold on; grid on;
    % Black-white friendly: distinct grayscale + line styles + markers
    grayLevels = [0.0, 0.5];
    lineStylesMeas = {':', '--'};  % measured: dotted/dashed (thin)
    lineStylesEst = {'-', '-.'};   % estimated: solid/dash-dot (thick)
    markers = {'o', 's'};
    markerInterval = 40;
    
    for i = 1:size(estCtrlFiles, 1)
        filename = estCtrlFiles{i, 1};
        label = estCtrlFiles{i, 2};
        startRow = estCtrlFiles{i, 3};
        endRow   = estCtrlFiles{i, 4};
        
        if exist(filename, 'file')
            Tall = readQRCData(filename);
            n = height(Tall);
            i1 = max(1, startRow);
            i2 = min(n, endRow);
            T = Tall(i1:i2, :);
            t = (T.Time - T.Time(1)) / 1000;
            grayColor = grayLevels(i) * [1 1 1];
            
            % Measured: thin line, no marker
            plot(t, T.distance, 'Color', grayColor, 'LineStyle', lineStylesMeas{i}, 'LineWidth', 1.2, ...
                 'DisplayName', sprintf('Measured (%s)', label));
            % Estimated: thick line with marker
            plot(t, -T.xhat, 'Color', grayColor, 'LineStyle', lineStylesEst{i}, 'LineWidth', 2, ...
                 'Marker', markers{i}, 'MarkerIndices', 1:markerInterval:length(t), ...
                 'MarkerSize', 5, 'MarkerFaceColor', grayColor, ...
                 'DisplayName', sprintf('Estimated (%s)', label));
        else
            warning('File not found: %s', filename);
        end
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Estimator + Controller: measured vs estimated distance');
    legend('Location','best');
end

%% ========================================================================
%  THEORETICAL ANALYSIS: FULL CLOSED-LOOP POLES
%  ========================================================================
fprintf('\n========== THEORETICAL ANALYSIS ==========\n\n');

fprintf('1(c) Closed-loop pole (state feedback, no estimator):\n');
fprintf('     p_cl = 1 - Ts*r*K = 1 - %.4f*K\n', Bd);
for i = 1:length(K_sweep)
    fprintf('     K = %3d rad/(s*m) -> p_cl = %.4f\n', K_sweep(i), 1 - Bd*K_sweep(i));
end
fprintf('     Stability: |p_cl| < 1 requires K < %.1f rad/(s*m)\n', 2/Bd);

fprintf('\n1(d) Estimator pole:\n');
fprintf('     p_est = 1 + L\n');
for i = 1:length(L_sweep)
    fprintf('     L = %.2f -> p_est = %.2f\n', L_sweep(i), 1 + L_sweep(i));
end
fprintf('     Stability: |p_est| < 1 requires -2 < L < 0\n');

fprintf('\n2(c) Full closed-loop poles (estimator + controller):\n');
fprintf('     Controller pole: p_cl = %.4f (K = %d)\n', p_cl_nom, K_nom);
fprintf('     Estimator pole (10x slower): p_est = %.4f (L = %.5f)\n', p_est_slow, L_slow);
fprintf('     Separation principle: poles are {%.4f, %.4f}\n', p_cl_nom, p_est_slow);

% Time constants
tau_cl = -Ts / log(p_cl_nom);
tau_est_slow = -Ts / log(p_est_slow);
fprintf('\n     Time constants:\n');
fprintf('       Controller: tau_cl = %.3f s\n', tau_cl);
fprintf('       Slow estimator: tau_est = %.3f s (ratio = %.1f)\n', tau_est_slow, tau_est_slow/tau_cl);

%% ========================================================================
%  HELPER FUNCTION: READ QRC DATA
%  ========================================================================
function T = readQRCData(filename)
% Read QRC CSV file and return table with named columns
% QRC format: header line, column names, then data
    
    % Read raw data, skipping header
    opts = detectImportOptions(filename);
    opts.DataLines = [3 Inf];  % Skip first 2 lines (header + column names)
    opts.VariableNamesLine = 2;
    
    rawData = readtable(filename, opts);
    
    % Map to meaningful names
    T = table();
    T.Time = rawData.Time;
    T.reference = rawData.ValueIn0;
    T.distance = rawData.ValueIn1;
    T.xhat = rawData.ValueIn2;
    T.v_ref = rawData.ValueIn3;
    T.speedA = rawData.ValueIn4;
    T.speedB = rawData.ValueIn5;
    T.voltA = rawData.ValueIn6;
    T.voltB = rawData.ValueIn7;
    T.nu = rawData.ValueIn8;
    T.xref = rawData.ValueIn9;
end
