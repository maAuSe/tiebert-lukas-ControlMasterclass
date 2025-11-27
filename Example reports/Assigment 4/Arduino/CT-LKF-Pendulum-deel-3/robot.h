#ifndef ROBOT_H
#define ROBOT_H

/*
 * ROBOT Class
 *
 * Class incorporating the robot. This class is used to define state machines,
 * control algorithms, sensor readings,...
 * It should be interfaced with the communicator to send data to the world.
 *
 */

#include "mecotron.h" // Include MECOTRON header
#include <BasicLinearAlgebra.h> // Include BasicLinearAlgebra to make matrix manipulations easier
#include "linear_kalman_filter.h" // Include template to make linear Kalman filter implementation easier

#define PENDULUM
#include <trajectory.h> // Include trajectory, for assignment 4

class Robot : public MECOtron {
  private:

    // Class variables
    Trajectory trajectory; // define the reference trajectory object

    // Kalman filter
    Matrix<3> _xhat;       // state estimate vector
    Matrix<3,3> _Phat;     // state estimate covariance
    Matrix<2> _nu;         // innovation vector
    Matrix<2,2> _S;        // innovation covariance

    // Position controller
    Matrix<1> ref;        // reference state
    Matrix<1> desiredVelocityCart;  // control signal

    float errorA[1] = {0.0};
    float errorB[1] = {0.0};
    float controlA[1] = {0.0};
    float controlB[1] = {0.0};

    float uA;
    float uB;


    
  public:
    // Constructor
    Robot() { }

    void control();

    // General functions
    bool init();  // Set up the robot

    bool controlEnabled();
    bool KalmanFilterEnabled();

    void resetController();
    void resetKalmanFilter();

    void button0callback();
    void button1callback();
    void button2callback();
    void button3callback();

};

#endif // ROBOT_H
