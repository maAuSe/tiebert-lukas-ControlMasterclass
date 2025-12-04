%% Assignment 3 - State feedback and state estimation
% Pole-placement design, simulation scaffolding, and plotting utilities.
% Populate the "data" folder with experiment CSV files and re-run this
% script to regenerate the figures referenced in the LaTeX report.

clear; close all; clc;

%% Configuration
Ts = 0.01;                    % s (scheduler period)
r = 0.033;                    % m (wheel radius)
A = 1;                        % discrete-time position state
B = Ts * r;                   % input gain (m/rad)
C = -1; D = 0;                % measurement: distance = -x
outDir = fullfile('C:\Users\campa\Documents\Arduino', ...
                  'tex_control', 'ass3_tex', 'images');
if ~exist(outDir, 'dir'); mkdir(outDir); end

% Controller/estimator sweeps (update if retuned)
K_sweep = [80, 120, 250];         % rad/(s*m)
L_sweep = [-0.05, -0.18, -0.35];  % observer gains for 2(a)
K_nom = 120;                       % nominal controller gain
L_slow = -0.004;                   % 10x slower estimator pole for 2(c)

%% 1(c) Closed-loop pole map vs K
p_cl = 1 - B * K_sweep;
figure('Name','pole_map_K'); hold on; grid on;
plot(real(p_cl), imag(p_cl), 'x', 'LineWidth', 1.5);
xline(1, '--k'); yline(0, ':k');
xlabel('Real'); ylabel('Imag'); title('Closed-loop pole vs K');
text(real(p_cl)+0.002, imag(p_cl), compose('K=%.0f', K_sweep));
exportgraphics(gcf, fullfile(outDir, 'pole_map_K.pdf'));

%% 1(c) Simulated position step responses for K sweep
Tsim = 4;                       % s
t = 0:Ts:Tsim;
x_true0 = -0.30;                % m (start 30 cm from wall)
x_ref = -0.15;                  % m (target 15 cm from wall)

figure('Name','step_K_sweep'); hold on; grid on;
for kVal = K_sweep
    xk = zeros(size(t));
    xk(1) = x_true0;
    for k = 1:(numel(t)-1)
        xk(k+1) = (1 - B * kVal) * xk(k) + B * kVal * x_ref;
    end
    plot(t, -xk, 'DisplayName', sprintf('K = %.0f', kVal)); % plot distance (= -x)
end
plot(t, -x_ref*ones(size(t)), 'k--', 'DisplayName', 'Reference');
xlabel('Time [s]'); ylabel('Distance to wall [m]');
title('Simulated distance step vs K (ideal velocity loop)');
legend('Location','best');
exportgraphics(gcf, fullfile(outDir, 'step_K_sweep.pdf'));

%% 1(d) Estimator pole vs L (theory)
p_est = 1 + L_sweep; % because A - L*C = 1 + L
figure('Name','pole_map_L'); hold on; grid on;
plot(real(p_est), imag(p_est), 'o', 'LineWidth', 1.5);
xline(1, '--k'); yline(0, ':k');
xlabel('Real'); ylabel('Imag');
title('Estimator pole vs L (p = 1 + L)');
text(real(p_est)+0.002, imag(p_est), compose('L=%.2f', L_sweep));
exportgraphics(gcf, fullfile(outDir, 'pole_map_L.pdf'));

%% 2(a) Estimator-only convergence (simulated placeholder)
est_init = -0.50;    % wrong initial xhat [m]
x_static = x_true0;  % cart not moving (u = 0)

figure('Name','estimator_L_sweep'); hold on; grid on;
for Lval = L_sweep
    xhat = zeros(size(t));
    xhat(1) = est_init;
    for k = 1:(numel(t)-1)
        % Prediction (u = 0) then correction with y = -x_static
        xhat_pred = xhat(k); % A=1, B=0
        nu = (-x_static) - (C * xhat_pred);
        xhat(k+1) = xhat_pred + Lval * nu;
    end
    plot(t, -xhat, 'DisplayName', sprintf('L = %.2f', Lval));
end
plot(t, -x_static*ones(size(t)), 'k--', 'DisplayName','Measured distance');
xlabel('Time [s]'); ylabel('Distance estimate [m]');
title('Estimator convergence vs L (simulated, no motion)');
legend('Location','best');
exportgraphics(gcf, fullfile(outDir, 'estimator_L_sweep.pdf'));

%% Helper: load experimental runs (fill file names when data available)
% Expected CSV columns: time, distance, xhat, innovation, speedA, speedB,
% voltA, voltB, v_ref
estOnlyRuns = {};         % e.g., {'data/est_only_Ln018.csv', ...}
ctrlOnlyRuns = {};        % e.g., {'data/ctrl_only_K120.csv', ...}
estCtrlRuns  = {};        % e.g., {'data/est_ctrl_good.csv','data/est_ctrl_bad.csv'}

% Uncomment and edit once data is available:
% estOnlyRuns = {'data/est_only_Ln018.csv'};
% ctrlOnlyRuns = {'data/ctrl_only_K120.csv'};
% estCtrlRuns  = {'data/est_ctrl_good.csv','data/est_ctrl_bad.csv'};

if ~isempty(estOnlyRuns)
    plotEstimatorOnly(estOnlyRuns, outDir);
end
if ~isempty(ctrlOnlyRuns)
    plotControllerOnly(ctrlOnlyRuns, outDir);
end
if ~isempty(estCtrlRuns)
    plotEstimatorController(estCtrlRuns, outDir);
end

%% Export suggested Arduino gains for traceability
fprintf('Suggested Arduino gains:\n');
fprintf('  K_nom     = %.1f rad/(s*m) -> p_cl = %.4f\n', K_nom, 1 - B*K_nom);
fprintf('  L_nom     = %.2f (estimator pole = %.2f)\n', L_sweep(2), 1 + L_sweep(2));
fprintf('  L_slow    = %.4f (10x slower, estimator pole = %.4f)\n', L_slow, 1 + L_slow);

%% ------------------------------------------------------------------------
function plotEstimatorOnly(files, outDir)
% Plot measured vs estimated distance for estimator-only runs.
    figure('Name','estimator_only_exp'); hold on; grid on;
    for idx = 1:numel(files)
        T = readtable(files{idx});
        plot(T.time, T.distance, '--', 'DisplayName', sprintf('meas %d', idx));
        plot(T.time, T.xhat, 'LineWidth', 1.2, ...
             'DisplayName', sprintf('est %d', idx));
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Estimator-only experiment (distance vs estimate)');
    legend('Location','best');
    exportgraphics(gcf, fullfile(outDir, 'estimator_L_sweep.pdf'));
end

function plotControllerOnly(files, outDir)
% Plot position step responses and control signals without estimator.
    figure('Name','controller_only_resp'); hold on; grid on;
    for idx = 1:numel(files)
        T = readtable(files{idx});
        plot(T.time, T.distance, 'DisplayName', sprintf('resp %d', idx));
        if ismember('reference', T.Properties.VariableNames)
            plot(T.time, T.reference, '--', ...
                 'DisplayName', sprintf('ref %d', idx));
        end
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Controller-only position steps');
    legend('Location','best');
    exportgraphics(gcf, fullfile(outDir, 'controller_K_response.pdf'));

    figure('Name','controller_only_voltage'); hold on; grid on;
    for idx = 1:numel(files)
        T = readtable(files{idx});
        if ismember('voltA', T.Properties.VariableNames)
            plot(T.time, T.voltA, 'DisplayName', sprintf('u_A %d', idx));
        end
        if ismember('voltB', T.Properties.VariableNames)
            plot(T.time, T.voltB, '--', 'DisplayName', sprintf('u_B %d', idx));
        end
    end
    xlabel('Time [s]'); ylabel('Voltage [V]');
    title('Controller-only control signals');
    legend('Location','best');
    exportgraphics(gcf, fullfile(outDir, 'controller_K_voltage.pdf'));
end

function plotEstimatorController(files, outDir)
% Plot measured vs estimated distance with controller + estimator.
    colors = lines(numel(files));
    figure('Name','est_ctrl'); hold on; grid on;
    for idx = 1:numel(files)
        T = readtable(files{idx});
        plot(T.time, T.distance, '--', 'Color', colors(idx,:), ...
             'DisplayName', sprintf('meas %d', idx));
        if ismember('xhat', T.Properties.VariableNames)
            plot(T.time, T.xhat, 'Color', colors(idx,:), ...
                 'DisplayName', sprintf('est %d', idx));
        end
    end
    xlabel('Time [s]'); ylabel('Distance [m]');
    title('Estimator + controller: measured vs estimated distance');
    legend('Location','best');
    exportgraphics(gcf, fullfile(outDir, 'est_ctrl_good.pdf'));

    % Duplicate export with alternate file name if two runs are provided
    if numel(files) >= 2
        exportgraphics(gcf, fullfile(outDir, 'est_ctrl_bad.pdf'));
    end
end
