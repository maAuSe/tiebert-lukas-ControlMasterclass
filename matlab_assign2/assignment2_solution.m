%% Assignment 2 - Velocity control of the cart
% This script implements the full workflow requested in specs_assignment2.md:
%   1. Build the discrete-time motor models that were selected in assignment 1.
%   2. Design two PI controllers (nominal and low-bandwidth) per motor using the
%      frequency response method.
%   3. Verify the design in the frequency and time domain (Bode, margins, steps).
%   4. Post-process the experimental data once it is recorded and generate the
%      required plots for Sections 2(a)–2(c) of the report.
%
% The script only relies on Assignment 1 data/models and Matlab's Control
% System Toolbox. Update the file paths in DATA CONFIGURATION once new
% experiments are logged.

clear; close all; clc;
format short g;

%% --- CONFIGURATION --------------------------------------------------------
Ts = 0.01;                        % sampling time used on the cart [s]
sampleRate = 1 / Ts;

designSpecNominal = struct( ...
    'tag',          'nominal', ...
    'description',  'High-bandwidth PI design for steady ground tests', ...
    'phaseMargin',  55, ...       % [deg]
    'phaseLagReserve', 15, ...    % safety margin for integrator lag [deg]
    'targetWc',     30);          % [rad/s] desired crossover

designSpecLowBW = struct( ...
    'tag',          'lowBandwidth', ...
    'description',  'Low-bandwidth PI (~0.5 Hz) for incline experiment', ...
    'phaseMargin',  55, ...
    'phaseLagReserve', 15, ...
    'targetWc',     2 * pi * 0.5); % [rad/s] ~ 0.5 Hz

exportPlots = false;              % set true to automatically save every figure
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir); scriptDir = pwd; end
% Explicitly point to the location where the experiment CSV files live.
dataDir = 'C:\Users\campa\Documents\Arduino\matlab_assign2\data';
figDir = fullfile(scriptDir, 'figures_assignment2');
if exportPlots && ~exist(figDir, 'dir')
    mkdir(figDir);
end

dataPaths = struct( ...
    'flatStep',          fullfile(dataDir, 'cart_flat_step.csv'), ...        % spec 2(a)
    'inclineNominal',    fullfile(dataDir, 'cart_incline_nominal.csv'), ...  % spec 2(b)
    'inclineLowBand',    fullfile(dataDir, 'cart_incline_lowband.csv'));     % spec 2(c)

%% --- MODELS FROM ASSIGNMENT 1 ---------------------------------------------
% Discrete-time models as reported in tex_control/assignment1.tex (unfiltered,
% simplified second-order fits with microOS delay). Each numerator includes
% the z^-2 term that reflects the relative degree dictated by the recursive ARX
% structure. Do not use the numbers from the example reports!

motorModels = struct();
motorModels.A = struct( ...
    'label', 'Wheel A', ...
    'discrete', tf([0, 0, 0.6309], [1, -0.6819, 0], Ts), ...
    'color', [0.10 0.33 0.60]);
motorModels.B = struct( ...
    'label', 'Wheel B', ...
    'discrete', tf([0, 0, 0.6488], [1, -0.6806, 0], Ts), ...
    'color', [0.80 0.32 0.07]);

motorNames = fieldnames(motorModels);
for k = 1:numel(motorNames)
    name = motorNames{k};
    motor = motorModels.(name);
    motor.discrete.InputName = "Voltage";
    motor.discrete.OutputName = "Velocity";
    motor.continuous = d2c(motor.discrete, 'tustin');
    motor.continuous.InputName = motor.discrete.InputName;
    motor.continuous.OutputName = motor.discrete.OutputName;
    motorModels.(name) = motor;
end

%% --- PI CONTROLLER DESIGN -------------------------------------------------
for k = 1:numel(motorNames)
    name = motorNames{k};
    motor = motorModels.(name);
    controllers = struct();
    controllers.nominal = designPiController(motor, designSpecNominal, Ts);
    controllers.lowBandwidth = designPiController(motor, designSpecLowBW, Ts);
    motor.controllers = controllers;
    motorModels.(name) = motor;
end

fprintf('\nController summaries (continuous PI gains and discrete coefficients)\n');
for k = 1:numel(motorNames)
    name = motorNames{k};
    motor = motorModels.(name);
    fprintf('\n%s\n', motor.label);
    specs = fieldnames(motor.controllers);
    for s = 1:numel(specs)
        ctrl = motor.controllers.(specs{s});
        fprintf('  %s controller:\n', ctrl.meta.description);
        fprintf('    Target crossover: %.2f rad/s (%.2f Hz)\n', ...
            ctrl.meta.targetWc, ctrl.meta.targetWc / (2*pi));
        fprintf('    Achieved crossover: %.2f rad/s, PM %.1f deg\n', ...
            ctrl.margins.wcp, ctrl.margins.pm);
        fprintf('    Continuous gains: Kp = %.4f, Ki = %.4f\n', ctrl.Kp, ctrl.Ki);
        fprintf('    PI zero time constant Ti = %.4f s\n', ctrl.Ti);
        fprintf('    Discrete numerator  (z^{-1} form): [%s]\n', num2str(ctrl.discrete.num, ' %.6f'));
        fprintf('    Discrete denominator(z^{-1} form): [%s]\n', num2str(ctrl.discrete.den, ' %.6f'));
        fprintf('    Difference eq: u[k] =%s\n', ctrl.discrete.textForm);
    end
end

%% --- FREQUENCY RESPONSE VISUALIZATION -------------------------------------
for k = 1:numel(motorNames)
    name = motorNames{k};
    motor = motorModels.(name);
    figName = sprintf('Plant Bode - %s', motor.label);
    plotPlantBode(motor.continuous, motor.label, figName, exportPlots, figDir);
    specs = fieldnames(motor.controllers);
    for s = 1:numel(specs)
        ctrl = motor.controllers.(specs{s});
        figName = sprintf('Open-loop Bode - %s - %s', motor.label, ctrl.meta.tag);
        plotOpenLoopBode(ctrl.openLoop, ctrl, figName, exportPlots, figDir);
    end
end

%% --- STEP RESPONSES (SIMULATION) ------------------------------------------
for k = 1:numel(motorNames)
    name = motorNames{k};
    motor = motorModels.(name);
    ctrlNom = motor.controllers.nominal;
    ctrlLow = motor.controllers.lowBandwidth;
    plotClosedLoopStepComparison(motor.label, ctrlNom, ctrlLow, exportPlots, figDir);
    plotControlSignalStepComparison(motor.label, ctrlNom, ctrlLow, exportPlots, figDir);
end

%% --- EXPERIMENTAL DATA PROCESSING -----------------------------------------
scenarioDefs = [ ...
    struct('key','flatStep', 'description','Flat ground step (spec 2a)', 'controllerTag','nominal'); ...
    struct('key','inclineNominal', 'description','Inclined plane step (spec 2b)', 'controllerTag','nominal') ...
];

experimentData = struct();
for s = 1:numel(scenarioDefs)
    def = scenarioDefs(s);
    filePath = dataPaths.(def.key);
    experimentData.(def.key) = loadQRCLog(filePath, Ts, def.description);
end
experimentData.inclineLowBand = loadQRCLog(dataPaths.inclineLowBand, Ts, ...
    'Inclined plane with low-bandwidth controller (spec 2c)');

% Section 2(a) and 2(b) plots
for s = 1:numel(scenarioDefs)
    def = scenarioDefs(s);
    data = experimentData.(def.key);
    if isempty(data)
        fprintf('[WARN] Missing data for scenario "%s". Skipping plots.\n', def.key);
        continue;
    end
    for k = 1:numel(motorNames)
        motorName = motorNames{k};
        motor = motorModels.(motorName);
        ctrl = motor.controllers.(def.controllerTag);
        plotExperimentalValidation(data, motorName, motor.label, ctrl, ...
            sprintf('%s - %s', def.description, motor.label), ...
            sprintf('%s_%s', def.key, motorName), exportPlots, figDir);
    end
end

% Section 2(c): overlay nominal vs low-bandwidth on incline
if ~isempty(experimentData.inclineNominal) && ~isempty(experimentData.inclineLowBand)
    for k = 1:numel(motorNames)
        motorName = motorNames{k};
        motor = motorModels.(motorName);
        plotScenarioComparison(experimentData.inclineNominal, motor.controllers.nominal, ...
            'Nominal controller', experimentData.inclineLowBand, ...
            motor.controllers.lowBandwidth, 'Low-bandwidth controller', ...
            motorName, motor.label, exportPlots, figDir);
    end
else
    fprintf('[WARN] Missing data for the incline comparison (spec 2c).\n');
end

%% --- BLOCK DIAGRAM FOR DISTURBANCE STUDY ----------------------------------
plotDisturbanceBlockDiagram(exportPlots, figDir);

fprintf('\nDone. Figures are available in Matlab. If exportPlots=true, they are saved under:\n  %s\n', figDir);

%% --- LOCAL FUNCTIONS ------------------------------------------------------
function controller = designPiController(motor, spec, Ts)
    arguments
        motor struct
        spec struct
        Ts double {mustBePositive}
    end

    w = logspace(log10(0.5), log10(500), 4000);
    [mag, phase] = bode(motor.continuous, w);
    mag = squeeze(mag);
    phase = squeeze(phase);
    if isempty(spec.targetWc)
        phaseTarget = -180 + spec.phaseMargin + spec.phaseLagReserve;
        idx = find(phase <= phaseTarget, 1, 'first');
        if isempty(idx)
            warning('Plant does not reach required phase for %s controller. Using highest available frequency %.2f rad/s.', spec.tag, w(end));
            idx = numel(w);
        end
        wc = w(idx);
    else
        wc = spec.targetWc;
    end
    Ti = tand(90 - spec.phaseLagReserve) / wc;
    D = tf([Ti 1], [Ti 0]);
    openLoopBase = D * motor.continuous;
    magAtWc = abs(evalfr(openLoopBase, 1i * wc));
    K = 1 / magAtWc;
    controllerC = K * D;
    openLoop = controllerC * motor.continuous;
    closedLoop = feedback(openLoop, 1);
    sensitivity = feedback(1, openLoop);
    controlTF = feedback(controllerC, motor.continuous);
    [gm, pm, wcg, wcp] = margin(openLoop);
    controller = struct();
    controller.meta = spec;
    controller.meta.targetWc = wc;
    controller.Ti = Ti;
    controller.Kp = K;
    controller.Ki = K / Ti;
    controller.continuous = controllerC;
    controller.openLoop = openLoop;
    controller.closedLoop = closedLoop;
    controller.sensitivity = sensitivity;
    controller.controlTF = controlTF;
    controller.margins = struct('gm', gm, 'pm', pm, 'wcg', wcg, 'wcp', wcp);
    controller.discrete.tf = c2d(controllerC, Ts, 'tustin');
    [num, den] = tfdata(controller.discrete.tf, 'v');
    controller.discrete.num = num;
    controller.discrete.den = den;
    controller.discrete.textForm = discreteDifferenceEquation(num, den);
end

function txt = discreteDifferenceEquation(num, den)
    assert(abs(den(1) - 1) < 1e-9, 'Denominator must be monic.');
    a = den(2:end);
    b = num;
    terms = [" "]; %#ok<NBRAK>
    terms(1) = sprintf(' %.6f*e[k]', b(1));
    for k = 2:numel(b)
        terms(end+1) = sprintf(' %.6f*e[k-%d]', b(k), k-1); %#ok<AGROW>
    end
    for k = 1:numel(a)
        terms(end+1) = sprintf(' %.6f*u[k-%d]', -a(k), k); %#ok<AGROW>
    end
    txt = strjoin(terms, ' + ');
end

function plotPlantBode(sys, label, figName, exportPlots, figDir)
    w = logspace(log10(0.3), log10(500), 1000);
    [mag, phase] = bode(sys, w);
    mag = squeeze(mag);
    phase = squeeze(phase);
    fig = figure('Name', figName, 'Color', 'w');
    subplot(2,1,1);
    semilogx(w, 20*log10(mag), 'LineWidth', 1.2);
    grid on; ylabel('Magnitude [dB]');
    title(sprintf('Bode of %s plant', label));
    subplot(2,1,2);
    semilogx(w, phase, 'LineWidth', 1.2);
    grid on; ylabel('Phase [deg]'); xlabel('Frequency [rad/s]');
    persistFigure(fig, figDir, figName, exportPlots);
end

function plotOpenLoopBode(openLoop, controller, figName, exportPlots, figDir)
    w = logspace(log10(0.3), log10(500), 1000);
    [mag, phase] = bode(openLoop, w);
    mag = squeeze(mag);
    phase = squeeze(phase);
    wcPlot = controller.margins.wcp;
    if isnan(wcPlot)
        wcPlot = controller.meta.targetWc;
    end
    pmText = controller.margins.pm;
    if isnan(pmText)
        pmText = controller.meta.phaseMargin;
    end
    fig = figure('Name', figName, 'Color', 'w');
    subplot(2,1,1);
    semilogx(w, 20*log10(mag), 'LineWidth', 1.3); hold on;
    yline(0, 'k--');
    xline(wcPlot, '--r', sprintf('\\omega_c = %.1f rad/s', wcPlot));
    grid on; ylabel('Magnitude [dB]');
    title(sprintf('Open-loop L(s) with %s controller', controller.meta.description));
    subplot(2,1,2);
    semilogx(w, phase, 'LineWidth', 1.3); hold on;
    xline(wcPlot, '--r');
    yline(-180 + controller.meta.phaseMargin, 'k:');
    grid on; ylabel('Phase [deg]'); xlabel('Frequency [rad/s]');
    text(wcPlot, -180 + pmText, sprintf('PM = %.1f deg', pmText), ...
        'VerticalAlignment','bottom');
    persistFigure(fig, figDir, figName, exportPlots);
end

function plotClosedLoopStepComparison(label, ctrlNom, ctrlLow, exportPlots, figDir)
    t = linspace(0, 3, 600);
    [yNom, tNom] = step(ctrlNom.closedLoop, t);
    [yLow, tLow] = step(ctrlLow.closedLoop, t);
    fig = figure('Name', sprintf('Closed-loop step response - %s', label), 'Color', 'w');
    plot(tNom, yNom, 'LineWidth', 1.5); hold on;
    plot(tLow, yLow, '--', 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Velocity [rad/s]');
    title(sprintf('Reference tracking - %s', label));
    legend(sprintf('Nominal (\\omega_c %.1f rad/s)', ctrlNom.margins.wcp), ...
           sprintf('Low-bandwidth (\\omega_c %.1f rad/s)', ctrlLow.margins.wcp), ...
           'Location', 'southeast');
    persistFigure(fig, figDir, sprintf('step_%s', label), exportPlots);
end

function plotControlSignalStepComparison(label, ctrlNom, ctrlLow, exportPlots, figDir)
    t = linspace(0, 3, 600);
    [uNom, tNom] = step(ctrlNom.controlTF, t);
    [uLow, tLow] = step(ctrlLow.controlTF, t);
    fig = figure('Name', sprintf('Control effort step - %s', label), 'Color', 'w');
    plot(tNom, uNom, 'LineWidth', 1.5); hold on;
    plot(tLow, uLow, '--', 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel('Voltage [V]');
    title(sprintf('Control signal on unit step - %s', label));
    legend('Nominal PI','Low-band PI','Location','southeast');
    persistFigure(fig, figDir, sprintf('control_step_%s', label), exportPlots);
end

function data = loadQRCLog(filePath, Ts, description)
    arguments
        filePath (1,:) char
        Ts double
        description (1,:) char
    end
    data = [];
    if ~isfile(filePath)
        fprintf('[INFO] Data file not found yet: %s\n', filePath);
        return;
    end
    raw = readmatrix(filePath, 'NumHeaderLines', 2);
    if isempty(raw)
        fprintf('[WARN] Empty dataset: %s\n', filePath);
        return;
    end
    timeVec = (raw(:,1) - raw(1,1)) * 1e-3; % convert ms to s
    dt = median(diff(timeVec));
    if abs(dt - Ts) > 5e-4
        fprintf('[INFO] Sample interval differs from design Ts: %.6f s vs %.6f s\n', dt, Ts);
    end
    data = struct();
    data.description = description;
    data.time = timeVec;
    % Column order in QRC CSV: [Time, ValueIn0, ValueIn1, ...]
    data.reference = raw(:, 2); % ValueIn0
    data.speedA = raw(:, 3);    % ValueIn1
    data.speedB = raw(:, 4);    % ValueIn2
    data.errorA = raw(:, 5);    % ValueIn3
    data.errorB = raw(:, 6);    % ValueIn4
    data.controlA = raw(:, 7);  % ValueIn5
    data.controlB = raw(:, 8);  % ValueIn6
    if size(raw,2) >= 9
        data.mode = raw(:, 9);  % ValueIn7
    end
end

function plotExperimentalValidation(data, motorKey, motorLabel, controller, titleStr, fileToken, exportPlots, figDir)
    t = data.time - data.time(1);
    ref = data.reference;
    switch motorKey
        case 'A'
            meas = data.speedA;
            ctrlMeas = data.controlA;
            errMeas = data.errorA;
        otherwise
            meas = data.speedB;
            ctrlMeas = data.controlB;
            errMeas = data.errorB;
    end
    simResponse = lsim(controller.closedLoop, ref, t);
    simError = lsim(controller.sensitivity, ref, t);
    simControl = lsim(controller.controlTF, ref, t);

    fig1 = figure('Name', sprintf('%s - closed loop', titleStr), 'Color', 'w');
    plot(t, ref, 'k--', 'LineWidth', 1.2); hold on;
    plot(t, meas, 'LineWidth', 1.3);
    plot(t, simResponse, 'LineWidth', 1.3);
    grid on; xlabel('Time [s]'); ylabel('Velocity [rad/s]');
    legend('Reference','Measured','Simulated','Location','southeast');
    title(sprintf('Closed-loop response - %s', motorLabel));
    persistFigure(fig1, figDir, sprintf('%s_cl_%s', fileToken, motorKey), exportPlots);

    fig2 = figure('Name', sprintf('%s - tracking error', titleStr), 'Color', 'w');
    plot(t, errMeas, 'LineWidth', 1.3); hold on;
    plot(t, simError, '--', 'LineWidth', 1.3);
    grid on; xlabel('Time [s]'); ylabel('Error [rad/s]');
    legend('Measured','Simulated','Location','southeast');
    title(sprintf('Tracking error - %s', motorLabel));
    persistFigure(fig2, figDir, sprintf('%s_err_%s', fileToken, motorKey), exportPlots);

    fig3 = figure('Name', sprintf('%s - control signal', titleStr), 'Color', 'w');
    plot(t, ctrlMeas, 'LineWidth', 1.3); hold on;
    plot(t, simControl, '--', 'LineWidth', 1.3);
    grid on; xlabel('Time [s]'); ylabel('Voltage [V]');
    legend('Measured','Simulated','Location','southeast');
    title(sprintf('Control effort - %s', motorLabel));
    persistFigure(fig3, figDir, sprintf('%s_ctrl_%s', fileToken, motorKey), exportPlots);
end

function plotScenarioComparison(dataNom, ctrlNom, labelNom, dataLow, ctrlLow, labelLow, motorKey, motorLabel, exportPlots, figDir)
    tNom = dataNom.time - dataNom.time(1);
    tLow = dataLow.time - dataLow.time(1);
    refNom = dataNom.reference;
    refLow = dataLow.reference;
    switch motorKey
        case 'A'
            measNom = dataNom.speedA; measLow = dataLow.speedA;
            errNom = dataNom.errorA; errLow = dataLow.errorA;
            uNom = dataNom.controlA; uLow = dataLow.controlA;
        otherwise
            measNom = dataNom.speedB; measLow = dataLow.speedB;
            errNom = dataNom.errorB; errLow = dataLow.errorB;
            uNom = dataNom.controlB; uLow = dataLow.controlB;
    end
    simNom = lsim(ctrlNom.closedLoop, refNom, tNom);
    simLow = lsim(ctrlLow.closedLoop, refLow, tLow);
    simErrNom = lsim(ctrlNom.sensitivity, refNom, tNom);
    simErrLow = lsim(ctrlLow.sensitivity, refLow, tLow);
    simCtrlNom = lsim(ctrlNom.controlTF, refNom, tNom);
    simCtrlLow = lsim(ctrlLow.controlTF, refLow, tLow);

    fig1 = figure('Name', sprintf('Incline comparison - %s', motorLabel), 'Color', 'w');
    plot(tNom, measNom, 'LineWidth', 1.4); hold on;
    plot(tLow, measLow, 'LineWidth', 1.4);
    plot(tNom, simNom, '--', 'LineWidth', 1.2);
    plot(tLow, simLow, '--', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Velocity [rad/s]');
    legend(sprintf('%s (meas)', labelNom), sprintf('%s (meas)', labelLow), ...
        sprintf('%s (sim)', labelNom), sprintf('%s (sim)', labelLow), ...
        'Location','southeast');
    title(sprintf('Incline tracking comparison - %s', motorLabel));
    persistFigure(fig1, figDir, sprintf('incline_compare_%s', motorKey), exportPlots);

    fig2 = figure('Name', sprintf('Incline tracking error - %s', motorLabel), 'Color', 'w');
    plot(tNom, errNom, 'LineWidth', 1.4); hold on;
    plot(tLow, errLow, 'LineWidth', 1.4);
    plot(tNom, simErrNom, '--', 'LineWidth', 1.2);
    plot(tLow, simErrLow, '--', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Error [rad/s]');
    legend(sprintf('%s (meas)', labelNom), sprintf('%s (meas)', labelLow), ...
        sprintf('%s (sim)', labelNom), sprintf('%s (sim)', labelLow), ...
        'Location','southeast');
    title(sprintf('Incline tracking error comparison - %s', motorLabel));
    persistFigure(fig2, figDir, sprintf('incline_err_compare_%s', motorKey), exportPlots);

    fig3 = figure('Name', sprintf('Incline control comparison - %s', motorLabel), 'Color', 'w');
    plot(tNom, uNom, 'LineWidth', 1.4); hold on;
    plot(tLow, uLow, 'LineWidth', 1.4);
    plot(tNom, simCtrlNom, '--', 'LineWidth', 1.2);
    plot(tLow, simCtrlLow, '--', 'LineWidth', 1.2);
    grid on; xlabel('Time [s]'); ylabel('Voltage [V]');
    legend(sprintf('%s (meas)', labelNom), sprintf('%s (meas)', labelLow), ...
        sprintf('%s (sim)', labelNom), sprintf('%s (sim)', labelLow), ...
        'Location','southeast');
    title(sprintf('Incline control effort comparison - %s', motorLabel));
    persistFigure(fig3, figDir, sprintf('incline_ctrl_compare_%s', motorKey), exportPlots);
end

function plotDisturbanceBlockDiagram(exportPlots, figDir)
    fig = figure('Name','Disturbance block diagram','Color','w','Position',[100 100 640 300]);
    axis off; hold on;
    controllerPos = [0.2 0.45 0.15 0.15];
    plantPos = [0.55 0.45 0.15 0.15];
    annotation('rectangle', controllerPos, 'FaceColor',[0.9 0.95 1], 'LineWidth',1.2);
    annotation('rectangle', plantPos, 'FaceColor',[0.9 1 0.9], 'LineWidth',1.2);
    annotation('textbox', controllerPos, 'String','C(z)','HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontWeight','bold','EdgeColor','none');
    annotation('textbox', plantPos, 'String','P(z)','HorizontalAlignment','center', ...
        'VerticalAlignment','middle','FontWeight','bold','EdgeColor','none');
    annotation('arrow',[0.1 0.2],[0.525 0.525],'LineWidth',1.2);
    annotation('arrow',[0.35 0.55],[0.525 0.525],'LineWidth',1.2);
    annotation('arrow',[0.70 0.85],[0.525 0.525],'LineWidth',1.2);
    annotation('arrow',[0.82 0.45],[0.45 0.25],'LineWidth',1.2);
    annotation('textarrow',[0.73 0.60],[0.76 0.60],'String','F_d (disturbance)','FontSize',9);
    annotation('textarrow',[0.92 0.82],[0.54 0.54],'String','y = \omega','FontSize',9);
    annotation('textarrow',[0.44 0.31],[0.25 0.25],'String','e = r - y','FontSize',9);
    annotation('ellipse',[0.12 0.48 0.06 0.09],'LineWidth',1.2);
    annotation('textbox',[0.12 0.6 0.1 0.1],'String','r','EdgeColor','none','FontWeight','bold');
    persistFigure(fig, figDir, 'disturbance_block_diagram', exportPlots);
end

function persistFigure(fig, figDir, baseName, exportPlots)
    if ~exportPlots
        return;
    end
    fileSafe = regexprep(lower(baseName), '\s+', '_');
    exportgraphics(fig, fullfile(figDir, [fileSafe '.png']), 'Resolution', 200);
end
