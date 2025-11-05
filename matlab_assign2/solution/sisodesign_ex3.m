
clear all
close all 
clear global
clc

%% 2.3 Choice and design of a suitable controller

% Define the transfer function of the system, that we will refer to as H. 
% To this end, first elaborate the denominator to the standard polynomial 
% form, or put it in the factorized form and use the 'zpk' command as
% follows:
H = zpk([],[0 -2 -5 -10],100);
figure; bode(H); title('H(s)'); 

% There is no steady-state error on a step reference, because there is an 
% integrator in H. Because there are no other specifications given in terms
% of actuator limitations or sensor noise, we can try to maximize the
% bandwidth with a lead compensator. Given that Mp < 0.2, we target a
% phase margin of 45°. The procedure is explained on slide 33 of C8.

PM = 45;                    % desired phase margin [°]
phi = -180 + PM - 55;       % the lead compensator will add 55° phase lead

% We then find the new cross-over frequency on the Bode diagram of H:
wc = 2.84;                  % new cross-over frequency [rad/s]

% Seeing that alpha = 0.1 is a standard choice, we have
alpha = 0.1; 

% and the break frequency becomes
T = 1/(wc*sqrt(alpha)); 

% Finally, we choose K such that the gain of the open loop equals 1 at the
% cross-over frequency wc. We can derive this gain by looking at the
% following Bode diagram:
D = tf([T 1],[alpha*T 1]);
figure; bode(D*H); title('D(s)*H(s), K = 1'); 

% We see we need an additional gain of 5.4 dB, so 
K = 10^(5.4/20);

% and the compensator becomes
D = K*D; 

% The closed loop becomes
CL = feedback(D*H,1);

% We can now verify whether our specifications are met in the frequency
% domain and in the time domain. 
figure; margin(D*H); title('Open loop: D(s)*H(s)'); 
figure; bode(CL); title('Closed loop: y/r'); 
figure; step(CL); title('Closed loop response of the output for a step reference input');

% Let us now apply the final value theorem to look for the steady-state
% error for a ramp. Verify on paper that the limit calculation boils down 
% to
ess = 100/(evalfr(D,0)*prod([2 5 10]));

% We can evaluate it in simulation by doing a simulation:
t = 0:0.1:10; t = t(:);
r = t; r = r(:);
y = lsim(CL,r,t);
figure; plot(t,r-y); title('Tracking error on a ramp reference (without feedforward)'); xlabel('time (s)');

% To eliminate this steady-state error, we can now add a feedforward gain
% Nv = 1/ess. The prefilter then becomes (see slide 96 of C8):
Np = 1;             % there is no steady-state error on a step
Nv = ess;           % there is a constant steady-state error on a ramp
prefilter = tf(Np*[Nv 1],1);

% In combination with the closed-loop, we get
CL_with_FF = CL*prefilter;

% and we can verify the steady-state error on a ramp now converges to 0:
y_with_FF = lsim(CL_with_FF,r,t);
figure; plot(t,r-y); hold on; plot(t,r-y_with_FF); title('Tracking error on a ramp reference (with feedforward)'); xlabel('time (s)');

% You can also derive the same result using the finite value theorem on
% paper again. Finally, if we compare the step responses with and without
% the feedforward, we notice that the overshoot of the response on the
% output on a step reference has become significantly larger:
figure; step(CL); hold on; step(CL_with_FF); title('Output on a step reference'); xlabel('time (s)'); legend('without feedforward','with feedforward');

% This is an important aspect to consider when you combine feedback and
% feedforward control systems: if you design them sequentially, which is
% the typical approach, the feedforward design will change the closed-loop
% behavior that you aimed for using the feedback design. Besides looking at
% the simulation of the step responses, we could have predicted this by
% looking at the poles and zeros of the closed loop with and without the
% feedforward: the feedforward introduces a zero at -1/ess. This additional
% zero influences the step response as indicated in slide 19 of C6. The
% dominant pole pair has a real part at -1.03 rad/s, and the zero is at 
% -0.90 rad/s. Hence, alpha = 0.87 and the zero does significantly
% influences the response and causes a much bigger overshoot than without
% the additional zero that is introduced by the velocity feedforward.