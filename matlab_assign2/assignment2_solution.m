%% Assignment 2 - Velocity control of the cart
% Simplified procedural script for PI controller design and validation.
% Covers sections 2(a), 2(b), and 2(c) of the report.

clear; close all; clc;

%% ========================================================================
%  ARDUINO COEFFICIENT CALCULATION - RUN THIS SECTION FIRST
%  ========================================================================
%  This section computes the discrete PI controller coefficients.
%  Copy the output to assignment2.cpp lines 13-18.
%  ========================================================================

%% Configuration
Ts = 0.01;
dataDir = '/Users/tiebertlefebure/Documents/Master of Mechanical Engineering/Control Theory/Arduino/matlab_assign2/data';

%% Plant models from .csvAssignment 1 (simplified 2nd-order model)
% H(z) = b1 / (z^2 + a1*z)  =>  tf([b1], [1, a1, 0], Ts)
wheelA_tf = tf([0.6309], [1, -0.6819, 0], Ts);
wheelB_tf = tf([0.6488], [1, -0.6806, 0], Ts);

% Convert to continuous for controller design
wheelA_cont = d2c(wheelA_tf, 'tustin');
wheelB_cont = d2c(wheelB_tf, 'tustin');

%% Show plant frequency response
figure; margin(wheelA_tf); title('Uncompensated open-loop system G_s(s) - Wheel A');
figure; margin(wheelB_tf); title('Uncompensated open-loop system G_s(s) - Wheel B');

%% ========================================================================
%  NOMINAL CONTROLLER (high bandwidth, ~30 rad/s crossover)
%  ========================================================================
wc_nom = 60;                          % target crossover [rad/s]
Ti_nom = tand(90 - 15) / wc_nom;      % integrator time constant (15 deg lag reserve)

% Compute gain K so that |L(j*wc)| = 1
D_nom = tf([Ti_nom 1], [Ti_nom 0]);
L_base_nom = D_nom * wheelA_cont;
K_nom = 1 / abs(evalfr(L_base_nom, 1i * wc_nom));

% Continuous and discrete PI controller
C_nom_cont = tf([K_nom, K_nom / Ti_nom], [1, 0]);
C_nom_disc = c2d(C_nom_cont, Ts, 'tustin');

% Open-loop, closed-loop, sensitivity, control TF
L_nom = C_nom_disc * wheelA_tf;
T_nom = L_nom / (1 + L_nom);
S_nom = 1 / (1 + L_nom);
U_nom = C_nom_disc / (1 + L_nom);

% Also design for Wheel B
L_base_nom_B = D_nom * wheelB_cont;
K_nom_B = 1 / abs(evalfr(L_base_nom_B, 1i * wc_nom));
C_nom_cont_B = tf([K_nom_B, K_nom_B / Ti_nom], [1, 0]);
C_nom_disc_B = c2d(C_nom_cont_B, Ts, 'tustin');

fprintf('Nominal controller (Wheel A):\n');
fprintf('  wc = %.2f rad/s, Ti = %.4f s, K = %.4f\n', wc_nom, Ti_nom, K_nom);
[num_nom_A, den_nom_A] = tfdata(C_nom_disc, 'v');
fprintf('  Discrete num: [%.6f, %.6f]\n', num_nom_A(1), num_nom_A(2));
fprintf('  Discrete den: [%.6f, %.6f]\n', den_nom_A(1), den_nom_A(2));

fprintf('Nominal controller (Wheel B):\n');
fprintf('  wc = %.2f rad/s, Ti = %.4f s, K = %.4f\n', wc_nom, Ti_nom, K_nom_B);
[num_nom_B, den_nom_B] = tfdata(C_nom_disc_B, 'v');
fprintf('  Discrete num: [%.6f, %.6f]\n', num_nom_B(1), num_nom_B(2));
fprintf('  Discrete den: [%.6f, %.6f]\n', den_nom_B(1), den_nom_B(2));

figure; margin(L_nom); title('Open-loop Bode - Nominal controller');

%% ========================================================================
%  LOW-BANDWIDTH CONTROLLER (~0.5 Hz = 3.14 rad/s crossover)
%  ========================================================================
wc_low = 2 * pi * 0.5;                % ~3.14 rad/s
Ti_low = tand(90 - 15) / wc_low;

D_low = tf([Ti_low 1], [Ti_low 0]);
L_base_low = D_low * wheelA_cont;
K_low = 1 / abs(evalfr(L_base_low, 1i * wc_low));

C_low_cont = tf([K_low, K_low / Ti_low], [1, 0]);
C_low_disc = c2d(C_low_cont, Ts, 'tustin');

L_low = C_low_disc * wheelA_tf;
T_low = L_low / (1 + L_low);
S_low = 1 / (1 + L_low);
U_low = C_low_disc / (1 + L_low);

% Also design for Wheel B
L_base_low_B = D_low * wheelB_cont;
K_low_B = 1 / abs(evalfr(L_base_low_B, 1i * wc_low));
C_low_cont_B = tf([K_low_B, K_low_B / Ti_low], [1, 0]);
C_low_disc_B = c2d(C_low_cont_B, Ts, 'tustin');

fprintf('\nLow-bandwidth controller (Wheel A):\n');
fprintf('  wc = %.2f rad/s, Ti = %.4f s, K = %.4f\n', wc_low, Ti_low, K_low);
[num_low_A, den_low_A] = tfdata(C_low_disc, 'v');
fprintf('  Discrete num: [%.6f, %.6f]\n', num_low_A(1), num_low_A(2));
fprintf('  Discrete den: [%.6f, %.6f]\n', den_low_A(1), den_low_A(2));

fprintf('Low-bandwidth controller (Wheel B):\n');
fprintf('  wc = %.2f rad/s, Ti = %.4f s, K = %.4f\n', wc_low, Ti_low, K_low_B);
[num_low_B, den_low_B] = tfdata(C_low_disc_B, 'v');
fprintf('  Discrete num: [%.6f, %.6f]\n', num_low_B(1), num_low_B(2));
fprintf('  Discrete den: [%.6f, %.6f]\n', den_low_B(1), den_low_B(2));

%% ========================================================================
%  ARDUINO COEFFICIENTS - COPY THESE TO assignment2.cpp (lines 13-18)
%  ========================================================================
fprintf('\n=== COPY TO assignment2.cpp ===\n\n');
fprintf('  coeffsA[MODE_NOMINAL]  = {%.6ff, %.6ff, 1.0f};\n', num_nom_A(1), num_nom_A(2));
fprintf('  coeffsB[MODE_NOMINAL]  = {%.6ff, %.6ff, 1.0f};\n', num_nom_B(1), num_nom_B(2));
fprintf('  coeffsA[MODE_LOW_BAND] = {%.6ff, %.6ff, 1.0f};\n', num_low_A(1), num_low_A(2));
fprintf('  coeffsB[MODE_LOW_BAND] = {%.6ff, %.6ff, 1.0f};\n', num_low_B(1), num_low_B(2));
fprintf('\n================================\n');

figure; margin(L_low); title('Open-loop Bode - Low-bandwidth controller');

%% Simulated step response comparison
t_sim = 0:Ts:5.5;
figure;
[y_nom, t_nom_step] = step(T_nom, t_sim);
[y_low, t_low_step] = step(T_low, t_sim);
plot(t_nom_step, y_nom, 'k-', 'LineWidth', 1.5); hold on;
plot(t_low_step, y_low, 'k--', 'LineWidth', 1.5);
legend('Nominal (30 rad/s)', 'Low-BW (3.14 rad/s)', 'Location', 'southeast');
title('Simulated closed-loop step response'); xlabel('Time [s]'); ylabel('Velocity [rad/s]');
grid on;

%% ========================================================================
%  SECTION 2(a): Flat ground step response
%  ========================================================================
csvfile_flat = fullfile(dataDir, 'cart_flat_step.csv');
if isfile(csvfile_flat)
    raw_flat = readmatrix(csvfile_flat, 'NumHeaderLines', 2);
    raw_flat = raw_flat(200:750, :);  % Clip to relevant range
    t_flat = 0:Ts:5.5; %(raw_flat(:,1) - raw_flat(1,1)) * 1e-3;
    ref_flat = raw_flat(:, 2);
    speedA_flat = raw_flat(:, 3);
    speedB_flat = raw_flat(:, 4);
    errorA_flat = raw_flat(:, 5);
    errorB_flat = raw_flat(:, 6);
    ctrlA_flat = raw_flat(:, 7);
    ctrlB_flat = raw_flat(:, 8);

    sim_flat = lsim(T_nom, ref_flat, t_flat);
    sim_err_flat = lsim(S_nom, ref_flat, t_flat);
    sim_ctrl_flat = lsim(U_nom, ref_flat, t_flat);

    % Wheel A
    figure;
    plot(t_flat, ref_flat, 'k:', 'LineWidth', 1.5); hold on;
    plot(t_flat, speedA_flat, 'k-', 'LineWidth', 1.2);
    plot(t_flat, sim_flat, 'k--', 'LineWidth', 1.2);
    legend('Reference', 'Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel A closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    figure;
    plot(t_flat, errorA_flat, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_flat, sim_err_flat, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel A tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    figure;
    plot(t_flat, ctrlA_flat, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_flat, sim_ctrl_flat, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel A control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;

    % Wheel B
    figure;
    plot(t_flat, ref_flat, 'k:', 'LineWidth', 1.5); hold on;
    plot(t_flat, speedB_flat, 'k-', 'LineWidth', 1.2);
    plot(t_flat, sim_flat, 'k--', 'LineWidth', 1.2);
    legend('Reference', 'Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel B closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    figure;
    plot(t_flat, errorB_flat, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_flat, sim_err_flat, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel B tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    figure;
    plot(t_flat, ctrlB_flat, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_flat, sim_ctrl_flat, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Flat ground - Wheel B control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;
else
    fprintf('[INFO] Flat ground data not found: %s\n', csvfile_flat);
end

%% ========================================================================
%  SECTION 2(b): Incline with nominal controller
%  ========================================================================
csvfile_incline = fullfile(dataDir, 'cart_incline_nominal.csv');
if isfile(csvfile_incline)
    raw_inc = readmatrix(csvfile_incline, 'NumHeaderLines', 2);
    raw_inc = raw_inc(164:404, :);  % Clip to relevant range
    t_inc = 0:Ts:2.4; %(raw_flat(:,1) - raw_flat(1,1)) * 1e-3;
    ref_inc = raw_inc(:, 2);
    speedA_inc = raw_inc(:, 3);
    speedB_inc = raw_inc(:, 4);
    errorA_inc = raw_inc(:, 5);
    errorB_inc = raw_inc(:, 6);
    ctrlA_inc = raw_inc(:, 7);
    ctrlB_inc = raw_inc(:, 8);

    sim_inc = lsim(T_nom, ref_inc, t_inc);
    sim_err_inc = lsim(S_nom, ref_inc, t_inc);
    sim_ctrl_inc = lsim(U_nom, ref_inc, t_inc);

    % Wheel A
    figure;
    plot(t_inc, ref_inc, 'k:', 'LineWidth', 1.5); hold on;
    plot(t_inc, speedA_inc, 'k-', 'LineWidth', 1.2);
    plot(t_inc, sim_inc, 'k--', 'LineWidth', 1.2);
    legend('Reference', 'Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel A closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    figure;
    plot(t_inc, errorA_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_inc, sim_err_inc, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel A tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    figure;
    plot(t_inc, ctrlA_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_inc, sim_ctrl_inc, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel A control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;

    % Wheel B
    figure;
    plot(t_inc, ref_inc, 'k:', 'LineWidth', 1.5); hold on;
    plot(t_inc, speedB_inc, 'k-', 'LineWidth', 1.2);
    plot(t_inc, sim_inc, 'k--', 'LineWidth', 1.2);
    legend('Reference', 'Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel B closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    figure;
    plot(t_inc, errorB_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_inc, sim_err_inc, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel B tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    figure;
    plot(t_inc, ctrlB_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_inc, sim_ctrl_inc, 'k--', 'LineWidth', 1.2);
    legend('Measured', 'Simulated', 'Location', 'southeast');
    title('Incline (nominal) - Wheel B control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;
else
    fprintf('[INFO] Incline nominal data not found: %s\n', csvfile_incline);
end

%% ========================================================================
%  SECTION 2(c): Incline comparison - nominal vs low-bandwidth
%  ========================================================================
csvfile_incline_low = fullfile(dataDir, 'cart_incline_lowband.csv');
if isfile(csvfile_incline) && isfile(csvfile_incline_low)
    raw_low = readmatrix(csvfile_incline_low, 'NumHeaderLines', 2);
    raw_low = raw_low(361:601, :);  % Clip to relevant range
    t_low = 0:Ts:2.4; %(raw_flat(:,1) - raw_flat(1,1)) * 1e-3;
    ref_low = raw_low(:, 2);
    speedA_low = raw_low(:, 3);
    speedB_low = raw_low(:, 4);
    errorA_low = raw_low(:, 5);
    errorB_low = raw_low(:, 6);
    ctrlA_low = raw_low(:, 7);
    ctrlB_low = raw_low(:, 8);

    sim_inc_nom = lsim(T_nom, ref_inc, t_inc);
    sim_inc_low = lsim(T_low, ref_low, t_low);
    sim_err_nom = lsim(S_nom, ref_inc, t_inc);
    sim_err_low = lsim(S_low, ref_low, t_low);
    sim_ctrl_nom = lsim(U_nom, ref_inc, t_inc);
    sim_ctrl_low = lsim(U_low, ref_low, t_low);

    % Wheel A - closed-loop comparison
    figure;
    plot(t_inc, speedA_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, speedA_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_inc_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_inc_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel A closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    % Wheel A - tracking error comparison
    figure;
    plot(t_inc, errorA_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, errorA_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_err_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_err_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel A tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    % Wheel A - control signal comparison
    figure;
    plot(t_inc, ctrlA_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, ctrlA_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_ctrl_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_ctrl_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel A control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;

    % Wheel B - closed-loop comparison
    figure;
    plot(t_inc, speedB_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, speedB_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_inc_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_inc_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel B closed-loop'); xlabel('Time [s]'); ylabel('Velocity [rad/s]'); grid on;

    % Wheel B - tracking error comparison
    figure;
    plot(t_inc, errorB_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, errorB_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_err_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_err_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel B tracking error'); xlabel('Time [s]'); ylabel('Error [rad/s]'); grid on;

    % Wheel B - control signal comparison
    figure;
    plot(t_inc, ctrlB_inc, 'k-', 'LineWidth', 1.2); hold on;
    plot(t_low, ctrlB_low, 'k-.', 'LineWidth', 1.2);
    plot(t_inc, sim_ctrl_nom, 'k--', 'LineWidth', 1);
    plot(t_low, sim_ctrl_low, 'k:', 'LineWidth', 1.5);
    legend('Nominal (meas)', 'Low-BW (meas)', 'Nominal (sim)', 'Low-BW (sim)', 'Location', 'southeast');
    title('Incline comparison - Wheel B control signal'); xlabel('Time [s]'); ylabel('Voltage [V]'); grid on;
else
    fprintf('[INFO] Missing data for incline comparison (section 2c).\n');
end

fprintf('Done.\n');
