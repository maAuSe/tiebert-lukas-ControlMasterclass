## 2.3 Assignment 3: state feedback and state estimation

In this assignment you will control the position of the cart which is driving on a straight line (i.e. both velocity controlled motors should get the same velocity setpoint $v$). To this end, a position control loop is added on top of the velocity controllers designed in assignment 2. For the position control you will design a state feedback controller. In order to retrieve an estimate of the state, you will implement a state estimator. This estimator uses distance measurements from the frontal infrared sensor to correct its estimate. The infrared sensor measures the distance to a wall in front of the cart. The range in which this sensor is accurate is limited (approximately 5-30 cm), so make sure the cart does not exceed those limits during your experiments to avoid strange results. As the cart is positioned at the negative side of the origin, the infrared sensor measures $-x$, which is a positive value. To model the cart, you can assume the velocity control loop is ideal, i.e. you assume that the desired velocity is equal to the real velocity of the cart.

[Figure 3: Diagram of cart measuring its distance to a wall.]
Figure 3: Cart measuring its distance to a wall.

For this assignment you can use the Arduino code provided on Toledo: `CT_StateEstimation_StateFeedback`. This code implements a working state estimator and applies a reference velocity signal to the motors to actuate the cart. You should extend the code by implementing the speed controller you designed in assignment 2. The comments in `robot.cpp` provide the body of an implementation and guide you where to add calls to your speed control routines. For your convenience, the position controller is already implemented, but you still need to enter your designed values for $K$ and $L$.

### 1. Design a state estimator and a state feedback controller using pole placement.

**(a) Write down the state equation in discrete form. The velocity of the cart can be seen as input. Use a forward Euler method as discretization scheme.**

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Write down the continuous-time state equation and show how you discretize it using the forward Euler discretization scheme. What are the $A$ and $B$ matrices of your state-space model?

**(b) Write down the measurement equation.**

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Write down the measurement equation. Use the $x$-axis convention of Fig. 3. What are the $C$ and $D$ matrices of your state-space model?

**(c) Design the state feedback controller gain $K$ using pole placement.**

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Derive an expression for the pole of the closed-loop system (assuming full state feedback, without estimator) as a function of the sample time $T_s$ and the state feedback gain $K$. How does it change as a function of $K$?
> * Make a pole-zero map and draw how the pole location changes with varying $K$.
> * Simulate a closed-loop step response for the same varying $K$. Plot all responses on the same figure, to compare the performance.
> * Explain how the properties of the time response for a step reference relate to the pole locations and to the value of $K$. Can we choose $K$ such that the closed-loop system becomes unstable?
> * Which value do you take for $K$, and why? Don't forget the units!

**(d) Design the state estimator gain $L$ using pole placement.**

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Derive an expression for the pole of the closed-loop estimator as a function of the sample time $T_s$ and the state estimator gain $L$. How does it change as a function of $L$? Can we choose $L$ such that the estimator becomes unstable?
> * How do you design appropriate values for $L$ when using pole placement? Which trade-offs do you take into account?
> * Give the value you obtained for $L$. Don't forget the units!

### 2. Implement and test a state estimator and a state feedback controller.

**(a) For this question, include the state estimator, but do not include the controller.** Use a wrong initial estimation for the position in the `resetStateEstimator`-routine in `robot.cpp`. Plot the evolution of the position estimate together with the measured position by the infrared sensor. Do this for different values of $L$. Explain the different responses and link the experimental results with the theoretical expected results from 1(d).

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Let the estimator start with a wrong initial estimate. Plot the measured distance together with the estimated distance for different values of $L$.
> * For which values of $L$ do the estimates converge faster to the measurements? Explain using the analysis and trade-offs treated in 1(d).
> * How does the value of $L$ influence the closed-loop estimator pole and the time response of the estimator?

**(b) For this question, do not use the state estimator, but do include the controller.** Directly use the position measurement to perform feedback on. Apply a step signal as position reference for different values of $K$. Illustrate the different step responses in one plot, and do the same for the corresponding control signals. Explain the plots using the analysis made in 1(c).

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Plot, on one figure, (i) the (position) step reference and (ii) its measured response for different values of $K$.
> * Do the properties of these time responses correspond with the theoretical expectation for each of the values of $K$? Verify and explain this, by comparing with the pole locations and corresponding step responses analyzed in 1(c).
> * Plot, on one figure, the low-level control signals (= voltage) that correspond to the responses of the previous figure. How do they depend on the value of $K$? Explain.
> * Are there theoretical and/or practical limitations on the choice of a feasible $K$? Which one(s)?

**(c) For this question, include both the estimator and the controller, such that the controller acts on the state estimates from the estimator.** Design a state estimator using pole placement, such that the closed-loop pole of the estimator is 10x *slower* than the closed-loop pole of the state feedback controller you have chosen. Perform an experiment in which the estimator starts while at the same time a step is applied as desired position signal. Plot the estimated position and measured position in one plot. Explain the observed behavior.

> **STRUCTURE YOUR ANSWER AS FOLLOWS:**
> * Start from a good position estimate and, at the same time, apply a step reference as the desired position. Plot the measured distance together with the estimated distance.
> * Start from a wrong position estimate and, at the same time, apply a step reference as the desired position. Plot the measured distance together with the estimated distance.
> * Is the control performance satisfactory in both cases? Why (not)? If you were to design a state estimator using pole placement yourself, where would you put its closed-loop pole?
> * Write down the poles of the full closed-loop system, including estimator and controller.
> * What do you notice when examining the poles of the full closed-loop system, and is this what you would expect? Does the estimator performance influence the control performance? Could you deduce that from the closed-loop transfer function and the closed-loop poles of the complete control system including estimator and controller? How / why not?