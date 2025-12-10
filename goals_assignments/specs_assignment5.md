# 2.5 Assignment 5: estimation and control of a two-wheel driven cart

This assignment considers the cart mounted with the swivel wheel and with two infrared sensors. Because the cart has two separately driven wheels, it acts as a two-wheel drive (2WD) system that can move in the horizontal plane. The system under consideration is shown in Figure 4. The 2WD cart moves around in the world's $XY$ plane. The coordinates of the cart's (geometric) center point are denoted by $(x_c, y_c)$, and $\theta$, the angle between the $X$-axis and the cart's longitudinal axis, determines the cart's orientation. A local coordinate system $X'Y'$ is attached to the cart at its center. The goal of this assignment is to estimate the cart's position and orientation $(x_c, y_c, \theta)$ by means of a Kalman filter and to use this estimate to make the cart follow a reference trajectory specified in terms of $(x_{c,ref}, y_{c,ref}, \theta_{ref})$. The trajectory involves three phases: firstly, a straight line has to be travelled at a constant forward velocity ($v_{ref} = 2$ cm/s) after which a turn has to be made at that same forward velocity with a constant rotational velocity ($\omega_{ref} = 0.44$ rad/s). Finally, a second straight line at that same forward velocity has to be tracked.

In this assignment you can again assume that the velocity control loop is ideal. To locate itself with respect to nearby objects, the cart is able to take distance measurements from two infrared sensors: one frontal, and one lateral (one side only). Without infrared measurements available, the cart's position can only be predicted based on the chosen model. This is known as *dead reckoning* and is subject to cumulative errors. When infrared measurements are available, a Kalman filter can correct the state estimation based on the first two statistical moments (mean and covariance) of states and measurements.

[Figure 4: Schematic overview of the robot moving in the world and measuring the distance to the walls. (x_c, y_c) are the coordinates of the robot's center in the world coordinate system XY. The robot's orientation is determined by theta, the angle between the X-axis and the robot's longitudinal axis X'. The dimensions a and alpha, beta, gamma are to be measured for the state equations and output equations respectively.]

**1. Model the system.**

(a) Write down the state equation. The input of the system can be chosen as $u = \begin{bmatrix} v \\ \omega \end{bmatrix}$, where $\omega$ [rad/s] denotes the rotational velocity of the robot around its center point $(x_c, y_c)$, and $v$ [m/s] is the robot's forward translational velocity (i.e. along its longitudinal axis $X'$). Choose the 2D pose of the cart, expressed in the world frame $XY$, as the state vector $\xi = [x_c, y_c, \theta]^T$ of the system.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Write down the relation between the forward velocity of the cart $v$, the rotational velocity of the cart $\omega$ and the velocity commands to both motors.
> * Write down the nonlinear continuous-time state-space equations ($\dot{\xi} = f(\xi, [v \ \omega]^T)$).

(b) Write down the measurement equation for both of the infrared sensors ($z_1, z_2$). Assume that a (straight) wall is characterized by $\mathcal{W} = \{(x,y) | px + qy = r\}$. The dimensions $\alpha, \beta, \gamma$ and $a$ can be measured on your platform if required.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Symbolically express the nonlinear measurement equations ($z = h(\xi, [v \ \omega]^T)$) in terms of $\alpha, \beta, \gamma$ and $a$, generally and specifically for the situation depicted in Figure 4.

---

**2. Derive and discuss the use of an extended Kalman filter.** Discretize your nonlinear continuous-time model and measurement equations using the forward Euler discretization scheme. Choose a state around which to linearize and linearize the discrete-time model. Discuss when and why you would choose a linear or extended Kalman filter and their respective differences.

(a) Discretize the continuous-time nonlinear model using the forward Euler method.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Discretize the nonlinear model. Clearly show which formulas you use to transform your continuous-time model into a discrete-time one.
> * Discretize the measurement equation. Clearly show which formulas you use to transform your continuous-time model into a discrete-time one.

(b) Linearize the discrete-time model around a state $\xi^*$.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Explain the meaning and choice of the state $\xi^*$ around which you linearize for the extended Kalman filter.
> * Discuss the differences between the choice of this state $\xi^*$ for the linear and extended Kalman filter. Are they equivalent? What is the effect on the performance of these estimators?
> * Derive the Jacobian of the discrete-time state and measurement equation for the situation depicted in Figure 4. Write down the linearized model. Clearly indicate the nonlinear equation(s) you start from, and show which assumptions/linearizations you apply to end up with a linear state-space model.

(c) Discuss the use of a linear or an extended Kalman filter.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Explain the differences between a linear and extended Kalman filter. What model do they use in their predictions and corrections? Is there any reason to assume that there is a difference between the process noise or measurement noise for both Kalman filters?
> * Discuss where each type of Kalman filter is more appropriate to use. Are they both equally performant or necessary for each type of model?

**3. Design and implement an extended Kalman filter.** You can use the Arduino code template that is provided on Toledo from here: `CT-EKF-Swivel`. Since this is a MIMO system, you will need to perform matrix manipulations during your calculations. If the template is not self-explanatory for you, you can find more information in the file `matrices.txt` in the example folder.

(a) Two sources of noise are incorporated in the model. Measurement noise is added to the output equation and is assumed to have a normal distribution with zero mean and a diagonal covariance matrix $R$. Process noise is added to the state equation and is assumed to have a normal distribution with zero mean and a diagonal covariance matrix $Q$. What are potential sources of process noise?

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * What are the potential sources of process noise and measurement noise specifically for the setup and model in this assignment? Which approximations have an influence on which type of noise?

(b) Design and tune an extended Kalman filter on the Arduino. Choose a reasonable value for the initial state estimate covariance $\hat{P}_{0|0}$. Implement the steps of the extended Kalman filter on the Arduino. Make sure the cart is located in $(-30, -20)$ cm (in the $XY$ frame, see Fig. 4). Then, apply a trajectory that makes a turn in the corner and stops at $(-15, -35)$ cm. This trajectory is built-in in the Kalman filter template. Have a look at the file `trajectory_swivel.txt` in the example folder of the template. The trajectory consists of two phases: one before the turn where both sensors are on and one during and after the turn where no sensors are used to collect measurements. Apply the feedforward inputs of this trajectory and plot the evolution of the estimated states. Explain how you have chosen $Q$ and $R$. Analyze and discuss the effect of varying $Q$ and $R$ on the behavior of the extended Kalman filter.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Explain the meaning of $Q$ and $R$. How do you choose reasonable values for them? Which tradeoffs do you take into account?
> * Choose sensible values for the initial state estimate covariance $\hat{P}_{0|0}$ and motivate your choice. Do not forget the units!
> * Implement the Kalman filter on your Arduino and cover the provided trajectory. Plot, on one figure, the evolution per state in time for varying $Q$ and $R$ values (max. 4 combinations). The number of figures is thus equal to the amount of states. Use the provided MATLAB function `plotstates` (see exercise session 5). To import your data from QRoboticsCenter for this specific assignment, you can call the method `KalmanExperiment.createfromQRC45()`. (*Hint: Play around with the ratio of Q and R.*)
> * What is the influence of $Q$ and $R$ on the behaviour of the extended Kalman filter? Does this correspond to the tradeoffs you expected? For which $\frac{Q}{R}$ ratios do the estimates converge faster to the measurements? Choose numerical values for $Q$ and $R$ and motivate your choice based on the plots. Do not forget their units!

(c) Discuss the uncertainty on the estimated states before and after the turn. Plot the evolution of the estimated states and their uncertainty, and the measurements and explain what you see in the plots based on the working principles of the Kalman filter.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Plot the evolution of the state estimates in time with their 95% confidence interval. To this end, use the provided MATLAB function `plotmeasurements` (see exercise session 5).
> * How does the uncertainty evolve for the different states? Is there a difference between the results for the different phases of the trajectory? Explain your answer based on the principles of the Kalman filter.
> * How do the measurement equations change if you switch off both sensors?
> * How do the measurement equations change before and during/after the turn? Consider hypothetically that the sensors would not be turned off. Describe in words what would change, a detailed derivation is not required.

**4. Design and implement a state feedback controller for following a position and orientation trajectory.** This controller takes as input the tracking error
$$\hat{e} = \begin{bmatrix} x_{c,ref} - \hat{x}_c \\ y_{c,ref} - \hat{y}_c \\ \theta_{ref} - \hat{\theta}_c \end{bmatrix}$$
In this part of the assignment, you will design, implement and tune an LQR controller based on a linearized model.

To design an LQR tracking controller, the error dynamics are derived in the local cart coordinate system X'Y': $\dot{e}' = R(\theta_c)(\dot{\xi}_{ref} - \dot{\xi}_c) + \dot{R}(\theta_c)(\xi_{ref} - \xi_c)$. In this assignment, we neglect the rotation over time of the cart frame $\dot{R}(\theta_c)$ and assume that this frame can be considered static. Expanding the error dynamics using the previously derived cart dynamics $\dot{\xi}_c = f(\xi_c, u)$, the following Jacobians of the error dynamics can be derived:
$$\frac{\partial \dot{e}'}{\partial \xi_c} = \begin{bmatrix} 0 & 0 & v_{ref} \sin(\theta_{ref} - \theta_c) \\ 0 & 0 & -v_{ref} \cos(\theta_{ref} - \theta_c) \\ 0 & 0 & 0 \end{bmatrix} \quad \frac{\partial \dot{e}'}{\partial u} = \begin{bmatrix} -1 & 0 \\ 0 & 0 \\ 0 & -1 \end{bmatrix}$$
As linearization point, we choose a point on the trajectory where all states and inputs of the cart equal the desired states and inputs of the reference trajectory ($\xi_{ref} = \xi_c$ and $\dot{\xi}_{ref} = \dot{\xi}_c$). This leads to the following continuous-time state equation matrices:
$$A = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & -v_{ref} \\ 0 & 0 & 0 \end{bmatrix} \quad B = \begin{bmatrix} -1 & 0 \\ 0 & 0 \\ 0 & -1 \end{bmatrix}$$
After discretization with the forward Euler method, the discrete-time state equation matrices are:
$$A_d = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & -T_s v_{ref} \\ 0 & 0 & 1 \end{bmatrix} \quad B_d = \begin{bmatrix} -T_s & 0 \\ 0 & 0 \\ 0 & -T_s \end{bmatrix}$$
Since we do not have access to the actual states of the system, all above equations in practice will contain the state estimate $\hat{\xi}$ instead of $\xi$. Except for $A_d$ and $B_d$, all equations above are continuous-time equations.

(a) Determine the rotation matrix that is required for the conversion of $(x, y, \theta)$, expressed in the world coordinate system $XY$, to $(x', y', \theta')$, expressed in the local coordinate system $X'Y'$. As this conversion depends on the orientation $\theta_{c,k}$ of the cart, use the estimate $\hat{\theta}_{c,k}$ to convert $\hat{e}_k$ to $\hat{e}'_k$.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Find the rotation matrix $R(\hat{\theta}_{c,k})$ such that $\hat{e}'_k = R(\hat{\theta}_{c,k})\hat{e}_k$.

(b) Derive the structure of the feedback matrix $K$ using Matlab.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Symbolically write down the structure of $Q$ and $R$ you use and explain your choice.
> * What are the sizes of $Q$ and $R$ and what is the meaning of their components?
> * Write the structure of the feedback matrix $K$ you obtain from Matlab. (*Hint: use the dlqr command.*)
> * Does this structure make sense? Why and what do the different components of this matrix stand for?

(c) Use a systematic approach to tune $Q$ and $R$.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * What is the meaning of Q and R in the LQR design objective?
> * Apply the reference trajectory to the closed-loop system and make a plot of the tracking errors of $x$, $y$ and $\theta$ for varying $Q$ and $R$ (again max. 4 combinations). Do not use any feedforward signals now (as opposed to the open-loop experiments of 2(c) where you exclusively applied feedforward signals).
> * Make a plot of the control signals $v$ and $\omega$ of the previous experiments.
> * Explain the effect of the different choices of $Q$ and $R$ on the observed controller behavior based on these plots. How does it influence the convergence speed of the controller? Are there limitations to be taken into account?
> * Report the chosen numerical value of Q and R and motivate your choice. Do not forget their units!