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
#include "state_estimator.h" // Include template to make state estimation implementation easier

using namespace BLA;

class Robot : public MECOtron {
  private:

    // Class variables

    struct PiCoeffs {
      float b0;
      float b1;
      float feedback;
    };

    struct PiState {
      float errorPrev;
      float controlPrev;
    };

    static constexpr float kTs = 0.01f;
    static constexpr float kWheelRadius = 0.033f;
    static constexpr float kVoltageLimit = 11.0f;
    static constexpr float kVelRefLimit = 25.0f;   // rad/s cap on commanded wheel speed
    static constexpr float kDefaultKpos = 40.0f;   // rad/(s*m)
    static constexpr float kDefaultX0 = -0.40f;    // m, used when no init value is provided

    PiCoeffs coeffA{0.945014f, -0.804389f, 1.0f}; // PI (assignment 2) - wheel A
    PiCoeffs coeffB{0.919504f, -0.782675f, 1.0f}; // PI (assignment 2) - wheel B
    PiState piStateA{0.0f, 0.0f};
    PiState piStateB{0.0f, 0.0f};
    float kPosGain = kDefaultKpos;
    float desiredDistanceMeters = 0.40f;  // positive distance to wall

    // State estimation
    Matrix<1> _xhat;      // state estimate vector
    Matrix<1> _nu;        // innovation vector

    // Position controller
    Matrix<1> xref;       // reference state
    Matrix<1,1> K;        // state feedback gain

  public:
    // Constructor
    Robot() { }

    void control();

    // General functions
    bool init();  // Set up the robot

    bool controlEnabled();
    bool StateEstimationEnabled();

    void resetController();
    void resetStateEstimator();

    float applyPi(float error, PiState &state, const PiCoeffs &coeffs) const;
    float saturate(float value, float limit) const;

    void button0callback();
    void button1callback();
    void button2callback();

};

#endif // ROBOT_H
