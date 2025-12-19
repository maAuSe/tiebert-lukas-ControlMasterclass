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
using namespace BLA;

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

    // Feedforward velocity commands for the cart
    Matrix<2> desiredVelocityCart;

    // LQR state feedback 
    Matrix<2,3> _Klqr;        // feedback gain matrix from dlqr(Ad, Bd, Q_lqr, R_lqr)
    Matrix<3> _errorBody;     // tracking error in body frame e' = R(theta_hat) * e_world

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
    void resetLqrController();

    // LQR helper: compute body-frame tracking error
    void computeBodyFrameError(float x_ref, float y_ref, float theta_ref,
                               float x_hat, float y_hat, float theta_hat,
                               Matrix<3> &e_body);

    void button0callback();
    void button1callback();
    void button2callback();
    void button3callback();

    float saturate(float value, float limit) const;
    float applyPi(float error, PiState &state, const PiCoeffs &coeffs) const;

};

#endif // ROBOT_H
