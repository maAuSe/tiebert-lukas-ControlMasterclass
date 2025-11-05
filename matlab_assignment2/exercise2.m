clear all
close all
clc

%% Data
% system: disc-spring/damper-disc system driven by a DC motor.
% input is voltage u, outputs angular positions of the two discs

% System parameters
I1 = 3;     % kgm^2
I2 = 3;     % kgm^2
c = 1;      % Nms/rad
k = 300;    % Nm/rad
c1 = 1;     % Nms/rad
c2 = 1;     % Nms/rad
L = 0.05;   % H
R = 2;      % Ohm
Km = 5;     % Vs/rad or Nm/A

%%  Define system:
% state space model with states: 
%   x1 = i (current in DC motor), 
%   x2 = theta_1, 
%   x3 = dtheta_1, 
%   x4 = theta_2,
%   x5 = dtheta_2
% output is theta_1
A = [-R/L 0 -Km/L 0 0; 0 0 1 0 0 ; Km/I1 -k/I1 -(c1+c)/I1 k/I1 c/I1; 0 0 0 0 1; 0 k/I2 c/I2 -k/I2 -(c+c2)/I2];
B = [1/L; 0; 0; 0; 0];
C = [0 1 0 0 0];
D = [0];

sys_G = ss(A, B, C, D);
[NumG, DenG] = ss2tf(A, B, C, D, 1);
DenG(end) = 0;
tfG = tf(NumG, DenG);

%% Define proportional controller with gain Kp=50
Kp = 50;
tfC = Kp;

% Define loop transfer function
tfL = tfC*tfG;

% Bode diagram 
figure(1)
bode(tfL)
grid on

% Nyquist diagram
figure(2)
h1 = nyquistplot(tfL); grid on
h1.showCharacteristic('AllStabilityMargins');
axis([-3 3 -3 3])
axis equal

% Stability margins
figure(3)
margin(tfL);
[Gm, Pm, w_gm, w_pm] = margin(tfL)

%% Stability of closed-loop system for Kp=50
% Calculate closed-loop system
tfCL = feedback(tfL,1);

% Calculate the pole of the closed loop system
pCL = pole(tfCL)

% Calculate step response and link damping to PM, wc to rise time/settling
% time
figure(4)
step(tfCL)
grid

% Calculate damping and Wn of poles
[wn,zeta] = damp(tfCL) 

% Link the overshoot of step response to zeta.
% Dominant poles are -0.813 +/- j 4.34, zeta = 0.18, wn = 4.42 rad/s
% Overshoot of step response (figure 4) is Mp = 0.5 
% Overshoot calculated from zeta of dominant pole is (slide 10, C6)
Mp = exp(-zeta(1)*pi/sqrt(1-zeta(1)^2))

% Why are these values not exactly equal?
% System is not a 2nd order system

% Link PM to overshoot
% See slide 27, C7: from figure we get that Pm = 23 is about Mp=0.55

% Link period of step response to wn
% Link wn to wc
% From figure 3 we get that wc = 4.14 rad/s
% This corresponds well with the frequency of the dominant pole: 4.42 rad/s
abs(pCL(4))

% From figure 4 we measure the period of the oscillation
% time of first peak: 0.702, time of second peak: 2.23
% period is Tp = 2.23-0.702 = 1.528
% w = 2*pi/Tp = 4.112

% What is steady state error on this step?  Why is it zero?
frd(tfCL,0)

% Check steady state error on unit ramp
t = (0:0.01:10)';
ramp = t;
y_ramp = lsim(tfCL,ramp,t);

figure(5)
plot(t, [t y_ramp])
grid
% Steady state error is about 0.116
% Calculated lim s->0 1/(s L(s))

tfL
% Divide numerator element corresponding to s^0 by denominator element
% Corresponding to s^1 (denominator element corresponding to s^0 is zero
% (because of integrator)
% 1.667e5/1.933e4
% the inverse of that is:
inv(1.667e5/1.933e4)
