clear all
close all
clear global
clc

%% 2.2 Design of a lead controller and a lag controller

% Define the transfer function of H. To this end, first elaborate the
% denominator to the standard polynomial form. 
H = tf(10, [1/15 17/30 1 0]); 

% Alternatively, you can also use the zero-pole-gain ('zpk') command of
% MATLAB. See its documentation for more details.

% PART 1: Lead design.
%%%%%%%%%%%%%%%%%%%%%%

% Calculate the K such that the steady-state specification is met, see
% slide 15 of C8, but by plugging in H instead of G in the formulas. You'll
% then find 10 * D(0) > 10, or D(0) = K >= 1.
K = 1; 

% Do the lead design according to slide 24 (and further) of C8. We
% determine the cross-over frequency of K*H as wc = 4.20 rad/s, and the
% associated phase is -184.17°. 
figure; bode(K*H); title('K*H(s)'); 
wc = 4.20;              % cross-over frequency of K*H [rad/s]
ph = -184.17;             % associated phase margin at wc of K*H [°]

% Calculate phimax based on the desired phase margin. We opt for a safety
% margin of 5°. 
SM = 5;                  % safety margin from 5 to 12, we choose 5 [°]
PM = 45;                 % from the figure on slide 16 of C8, we approximately know the desired phase margin for Mp = 2 [°]
phimax = -180 + PM - ph + SM; 

% We can now find alpha from the formula on slide 13 of C8:
alpha = (1-sind(phimax))/(1+sind(phimax)); 

% Now, we determine the new cross-over frequency of the open-loop as the
% frequency where the gain of K*H(s) equals sqrt(alpha) = 0.32, or, in dB,
20*log10(3.10)

% From the Bode diagram of K*H(s), we then find the new wc associated with 
% the gain of -9.90 dB as
wc = 6.90;              % new cross-over frequency [rad/s]

% The break frequency becomes 
T = 1/(wc*sqrt(alpha)); % time associated with the break frequency [s]

% Define the transfer function of the lead compensator. 
D_lead = tf([K*T K],[alpha*T 1]);

% Plot the open loop and validate whether the phase margin is indeed
% approximately 45°. 
figure; margin(D_lead*H); title('Open loop, lead compensator: D_{lead}(s)*H(s)'); 

% PART 2: Lag design.
%%%%%%%%%%%%%%%%%%%%%

% Calculate the K such that the steady-state specification is met, see
% slide 15 of C8, but by plugging in H instead of G in the formulas. You'll
% then find 10 * D(0) > 10, or D(0) = K >= 1.
K = 1; 

% Do the lag design according to slide 60 (and further) of C8. We look for
% the frequency where the phase equals the desired phase margin plus a 
% safety margin of 5°. 
PM = 45;                 % from the figure on slide 16 of C8, we approximately know the desired phase margin for Mp = 2 [°]
SM = 5; 
ph = PM + SM;

% We find the new cross-over frequency on the Bode diagram of K*H where the
% phase equals 50°: 1.31 rad/s. The associated gain is 16.4 dB = 6.61.
% To set this frequency as the new cross-over frequency, we thus have to
% choose alpha = 6.61. 
figure; bode(K*H); title('K*H(s)'); 
wc = 1.31;               % new cross-over frequency [rad/s]
alpha = 6.61;

% To put the zero a decade higher than the new cross-over frequency, we 
% have to choose T = 10/wc.
T = 10/wc; 

% Due to the fact that alpha also alters the gain of the compensator, we
% have to scale K with 1/alpha as to comply with the steady-state
% specifications: K = K/alpha.
K = K/alpha; 

% Define the transfer function of the lead compensator. 
D_lag = tf([K*alpha*T K*alpha],[alpha*T 1]);

% Plot the open loop and validate whether the phase margin is indeed
% approximately 45°. 
figure; margin(D_lag*H); title('Open loop, lag compensator: D_{lag}(s)*H(s)'); 

% COMPARISON
% %%%%%%%%%%

% Plot the Bode diagram of the open loop transfer functions.
figure; margin(D_lead*H); hold on; margin(D_lag*H); legend('Lead design','Lag design'); title('Open loop');

% Calculate the closed-loop transfer functions:
%   - from reference to output:
      r_to_y_lead = feedback(D_lead*H,1);
      r_to_y_lag = feedback(D_lag*H,1);
%   - from reference to control signal:
      r_to_u_lead = feedback(D_lead,H);
      r_to_u_lag = feedback(D_lag,H);

% Plot the Bode diagram of the closed-loop transfer functions.
figure; bode(r_to_y_lead); hold on; bode(r_to_y_lag); legend('Lead design','Lag design'); title('Closed loop: y/r');
figure; bode(r_to_u_lead); hold on; bode(r_to_u_lag); legend('Lead design','Lag design'); title('Closed loop: u/r');

% Note that at high frequencies, the frequency response from reference to
% controller output (control signal) of the lead compensator is way higher
% than the one of the lag compensator. The lead compensator would thus
% amplify noise and need a lot of actuator effort in that case, why the lag
% compensator wouldn't. However, the lag compensator is a lot slower. The
% preferred compensator type thus strongly depends on the application. 

% Simulate the step responses.
figure; step(r_to_y_lead); hold on; step(r_to_y_lag); legend('Lead design','Lag design'); title('Step response: output for a step reference');
figure; step(r_to_u_lead); hold on; step(r_to_u_lag); legend('Lead design','Lag design'); title('Step response: control signal for a step reference'); 

% Note that none of the above designs complies with the overshoot limit of
% 20%. That is, for both the lead and lag designs we would have to iterate
% a few teams by increasing the safety margins on the phase. 