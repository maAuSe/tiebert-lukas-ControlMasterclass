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

#include "mecotron.h" 
#include <BasicLinearAlgebra.h> 
#include "state_estimator.h" 

using namespace BLA;

class Robot : public MECOtron {
  private:


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
    static constexpr float kVelRefLimit = 25.0f;   
    static constexpr float kDefaultKpos = 40.0f;   
    static constexpr float kDefaultX0 = -0.40f;    

    PiCoeffs coeffA{0.945014f, -0.804389f, 1.0f}; 
    PiCoeffs coeffB{0.919504f, -0.782675f, 1.0f}; 
    PiState piStateA{0.0f, 0.0f};
    PiState piStateB{0.0f, 0.0f};
    float kPosGain = kDefaultKpos;
    float desiredDistanceMeters = 0.40f;  

    Matrix<1> _xhat;      
    Matrix<1> _nu;       

    Matrix<1> xref;    
    Matrix<1,1> K;      

  public:
    
    Robot() { }

    void control();

    bool init();  

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
