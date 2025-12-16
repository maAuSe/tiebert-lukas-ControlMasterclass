## 2.1 Assignment 1: identification of the cart

1. Select an appropriate model structure for the cart, for the dynamic relation between voltage applied to the DC-motors and the velocity of the cart measured by the wheel encoders. Each cart is equiped with two voltage controllerd DC-motors. The relation between the voltage and the velocity can be approximated as a linear model of a DC-motor with load and friction, assuming:

    * the same voltage is applied to both motors,
    * no wheel slip,
    * only forward and backward motions are considered (no turning)
    * cart mass and ground friction are lumped into the wheel inertia and friction respectively.

    (a) Select a discrete-time model structure that you will use for identification of the relation between motor voltage to rotational wheel velocity (**max. 1 page**).

    > **STRUCTURE YOUR ANSWER AS FOLLOWS:**
    >
    > * This selection is based on the physical laws and continuous-time transfer function describing the behavior of your system, simplifications you assume, the sampling process (zero-order-hold) and potential delay(s) introduced by the software framework MicroOS. Do not derive the physical laws but just present and discuss the continous-time transfer function relating voltage to velocity and denote all variable and parameters with their physical units. Then you present the discrete-time transfer function, clearly indicating the orders of the numerator and denominator and the number of delays.
    > * Briefly (max. 5 lines) explain how the discrete-time model was derived and briefly (max. 5 lines) motivate the choice(s) you made during this model structure selection. If you plan to try out more than one model structure, explain your motivation for this as well, but limit the number of different model structures to two.

    (b) Write down the recursion expression (difference equation) and criterion that you use to estimate the model parameters (**max. 1 page**).

    > **STRUCTURE YOUR ANSWER AS FOLLOWS:**
    >
    > * Write down the recursion expression(s) (difference equation(s)) specifically for your model(s) and write down the error criterion that you minimize to find the parameters.
    > * Write down the matrix formulation of the parameter estimation problem specifically for your/one of your model(s).

2. Identify the cart by exciting the motors while the cart is on the ground.

    (a) Which excitation signal do you apply? Why? (**max. half a page**)

    > **STRUCTURE YOUR ANSWER AS FOLLOWS:**
    >
    > * Motivate the choice of the excitation signal you selected/designed.
    > * Plot your excitation signal. Do you apply it just once or do you repeat it? Why?

    (b) Estimate the model parameters and validate the accuracy/characteristics of the model(s) for both motors (**max. 1 page**)

    > **STRUCTURE YOUR ANSWER AS FOLLOWS:**
    >
    > * Plot the measured response and response simulated with the identified model(s). Make sure that differences are visible (do not show the whole measured sequence, zoom in), also plot difference between measurement and simulation.
    > * Discuss the differences between simulation and measurement (transient and steady-state). Discuss transient and steady-state characteristics of the model(s) and compare them with those of the measurements/simulation. If you have identified multiple models, compare them and decide which model **structure/order** you prefer and will continue to use. Discuss the differences between the models of both motor and discuss wether these difference significant are significant or not (and motivate).

    (c) Verify if data filtering can improve the identification (**max. 1 page**)

    > **STRUCTURE YOUR ANSWER AS FOLLOWS:**
    >
    > * Why do you prefer to filter your signals before fitting a model? Which signals do you filter before fitting? Explain.
    > * How do you design the filter that you use?
    > * Write down the characteristics of the filter that you use (type, order, cut-off frequency, ...) and motivate your choices.
    > * Redo the identification after filtering your data.
    > * Plot the measured response and response simulated with the model identified with filtered data and the model of 2(b).
    > * Perform the same model validation as in 2(b) and conclude which model you prefer and will continue to use.