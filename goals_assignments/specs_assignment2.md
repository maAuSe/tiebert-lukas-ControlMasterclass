2.2 Assignment 2: velocity control of the cart

1. Design for each DC motor a velocity controller that yields zero steady-state error on a constant velocity reference. Design the controller using the frequency response method. Use the identified models (one for each motor) selected in part 2.

   (a) What type of controller do you choose? Why? (max. 1/4 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Write down the requirements/specifications that you would like to obtain from your controlled system. Which controller types can realize those?
      * Select one of the possible controller types and motivate your choice.

   (b) Explain the design process and all choices of design parameters (phase margin, cross-over frequency, integration time, ...). (max. 1 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Write down the formulas you use to compute the design parameters of the selected controller type in order to meet the requirements you listed in (a). Also write down the numerical values that you enter in these formulas, and the numerical values you obtain when evaluating them.
      * Briefly explain the design trade-offs involved in the controller design parameters. That is: for every parameter explain the advantage/disadvantage of increasing/decreasing its value.
      * Plot the open loop (= serial connection of your controller and your model) on a Bodediagram and clearly indicate *on this plot* where you see the characteristics that affect the choice of your design parameters.
      * Verify that the specifications are met in time and frequency domain by making appropriate plots.

   (c) Is there a theoretical limitation on the closed-loop bandwidth that can be achieved? Is there a practical limitation? (max. 1/4 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Is it theoretically possible to design a controller that yields a higher bandwidth? If not, explain the corresponding theoretical bandwidth limitations.
      * Do you expect / experience practical problems when increasing the closed-loop bandwidth of the controller? If so, explain the corresponding practical bandwidth limitations.
      * Does the way you implement your controller in practice (software + microcontroller) impose theoretical and/or practical constraints on the achievable bandwidth?

2. Validate the designed controller experimentally.

   (a) Implement the controller for both wheels on the Arduino and apply a step input for the reference velocity. Validate the controller. (max. 1 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Plot, on one figure, (i) the step reference, (ii) its measured closed-loop response and (iii) its simulated closed-loop response.
      * Plot the measured tracking error of the step reference together with the simulated tracking error.
      * Plot the measured control signal (= voltage) of the step reference together with the simulated control signal.
      * Discuss the abovementioned comparisons in detail. Also compare the performance characteristics (transient and steady-state) with designed characteristics.

   (b) Apply a constant force disturbance to the cart and validate your controller with respect to steady state performance. (max. 1 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Draw the block diagram of your control configuration, including the force disturbance signal, and clearly indicate where the disturbance is entering the loop. Hint: the dynamics of your cart behave like a DC motor with a load, which are schematically represented in the figure below. u is the input voltage, i the motor current, T the motor torque, $\dot{\theta}$ the motor speed, R the armature resistance, $J_{eq}$ the equivalent inertia projected on the motor shaft, the equivalent damping is $c_{eq}$, and $K_t = K_v$ is the motor constant.
      
      [Diagram of DC motor circuit and mechanical load]

      * Apply the same velocity setpoint as in 2(a). Make the same plots as in 2(a).
      * Is the controller still tracking the reference despite the disturbance? Explain why (not).

   (c) Repeat the controller design and select a cross-over frequency of about 0.5Hz. Validate this controller for the cart on a ramp and compare with the results of 2(b). (max. 1 page)

      **STRUCTURE YOUR ANSWER AS FOLLOWS:**
      * Make the same plots as in 2(b) including the results of 2(b).
      * Analyse and discuss the differences between the responses of both controllers. What is the contributions of the different parts of the controller in both cases?