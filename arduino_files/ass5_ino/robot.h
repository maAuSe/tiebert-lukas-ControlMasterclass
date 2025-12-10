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
#include "extended_kalman_filter.h" // Include template to make extended Kalman filter implementation easier

#define SWIVEL
#include <trajectory.h> // Include trajectory, for assignment 5

class Robot : public MECOtron {
  private:

    // Class variables
    Trajectory trajectory; // define the reference trajectory object

    struct PiCoeffs {
      float b0;
      float b1;
      float feedback;
    };

    struct PiState {
      float errorPrev;
      float controlPrev;
    };

    // Kalman filter
    Matrix<3> _xhat;       // state estimate vector
    Matrix<3,3> _Phat;     // state estimate covariance
    Matrix<2> _nu;         // innovation vector
    Matrix<2,2> _S;        // innovation covariance

    // Position controller
    Matrix<3> xref;        // reference state
    Matrix<2> desiredVelocityCart; // control signal
    Matrix<2,3> Kfb;       // state feedback gains (cart frame)

    // Velocity controller
    PiState piStateA;
    PiState piStateB;
    PiCoeffs piCoeffsA;
    PiCoeffs piCoeffsB;
    float kVoltageLimit = 11.0f;
    float kVelRefLimit = 12.0f;  // rad/s limit for wheel speed commands

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
    void resetVelocityController();

    void button0callback();
    void button1callback();
    void button2callback();
    void button3callback();

    float saturate(float value, float limit) const;
    float applyPi(float error, PiState &state, const PiCoeffs &coeffs) const;

};

#endif // ROBOT_H
