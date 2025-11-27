close all
clear all
clc

Ts = 0.01;
r = 0.033;
A = 1;
B = Ts*r;
C=-1;
ts = 10; % controller settling time
est_factor = 10;  % Settling time factor (how much faster than the controller). Typically 2 to 6 or higher, to be used as a tuning factor.
ts_est = ts / est_factor;  % Calculate the settling time for the estimator, which is a factor of the controller's settling time.
dzeta = 0.7;  % Damping factor for the estimator, typically between 0.5 and 1 for a stable response.
wn = 4.6/(ts_est*dzeta);  % Natural frequency for the estimator poles, based on settling time and damping factor.
sigma = dzeta*wn;  % Damping coefficient.
pc = -sigma; % Complex conjugate poles for estimator.


% controller
pc = exp((-4.6*Ts)/1);
K = place(A,B,pc)


% estimator 2 times faster
pd_2x = exp(Ts*pc*2);
L_2x = place(A',A'*C',pd_2x)

% estimator 3 times faster
pd_3x = exp(Ts*pc*3);
L_3x = place(A',A'*C',pd_3x)

% estimator 4 times faster
pd_4x = exp(Ts*pc*4);
L_4x = place(A',A'*C',pd_4x)

% estimator 6 times faster
pd_6x = exp(Ts*pc*6);
L_6x = place(A',A'*C',pd_4x)


% estimator 10 times slower 
pd_10 = exp(Ts*pc/10);
L_10xslower = place(A',A'*C',pd_10)

% Compute estimator poles for different speeds
pd_2x = exp(Ts * pc * 2);
pd_3x = exp(Ts * pc * 3);
pd_4x = exp(Ts * pc * 4);
pd_6x = exp(Ts * pc * 6);
pd_10 = exp(Ts * pc / 10);

% Plot poles on zero-pole map
figure;
hold on;
plot(real(pd_2x), imag(pd_2x), 'r*', 'DisplayName', '2x Estimator'); % Star for 2x
plot(real(pd_3x), imag(pd_3x), 'bo', 'DisplayName', '3x Estimator'); % Circle for 3x
plot(real(pd_4x), imag(pd_4x), 'gd', 'DisplayName', '4x Estimator'); % Diamond for 4x
plot(real(pd_6x), imag(pd_6x), 'ms', 'DisplayName', '6x Estimator'); % Square for 6x
plot(real(pd_10), imag(pd_10), 'kx', 'DisplayName', '10x Slower Estimator'); % Cross for 10x slower

% Plot unit circle
theta = linspace(0, 2*pi, 1000);
unit_circle_x = cos(theta);
unit_circle_y = sin(theta);
plot(unit_circle_x, unit_circle_y, 'k--', 'DisplayName', 'Unit Circle');

% Add labels and grid
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
title('Zero-Pole Map with Distinct Symbols');
legend;
axis equal;
hold off;