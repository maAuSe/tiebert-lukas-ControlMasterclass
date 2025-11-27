clear all
close all
clc

% Parameters
L = 0.135;    % Length of pendulum (example value)
g = 9.81; % Gravitational acceleration
c = 0;  % Damping coefficient
m = 1;    % Mass of the pendulum
Ts = 0.01; % Sampling time (example value)



% Continuous-time state-space matrices
A = [0, 0, 0;
     0, 0, 1/L;
     0, -g, -c/(m*L)];
B = [1;
     -1/L;
     -c/(m*L)];
C = [1, 0, 0;
     0, 1, 0];
D = [0; 0];
I = [1 0 0;
    0 1 0;
    0 0 1];
%Approach 1:
sys_disc1 = ss(I + A*Ts, Ts * B, C, D)

% Approach 2


sys_cont2 = ss(A, B, C, D);

sys_disc2 = c2d(sys_cont2, Ts, 'zoh')

%[Phi, Gamma, Cd, Dd] = ssdata(sys_disc2);


figure
step(sys_disc1,1:Ts:15)

figure
step(sys_disc2,1:Ts:15)

