/*
 * STATE ESTIMATION AND STATE FEEDBACK CONTROL TEMPLATE
 *
 * This is a template to get you started with the implementation of the state estimation and state feedback control
 * on your own cart.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); 

  xref.Fill(0);
  _xhat.Fill(0);
  _nu.Fill(0);
  desiredDistanceMeters = 0.40f;  

  // EXPERIMENT GAIN CONFIGURATION 
  //
  // Section 2(a) Estimator-only: L sweep (controller OFF, estimator ON) 
  // setEstimatorGain(-0.05f);      // L_slow:  p_est ~= 0.95 (estimator-only sweep)
  // setEstimatorGain(-0.18f);      // L_nom:   p_est ~= 0.82
  // setEstimatorGain(-0.35f);      // L_fast:  p_est ~= 0.65
  //
  // Section 2(b) Controller-only: K sweep (controller ON, estimator OFF) 
  // kPosGain = 20.0f;              // K_slow: p_cl ~= 0.993
  kPosGain = 40.0f;                 // K_nom:  p_cl ~= 0.987  <-- default
  // kPosGain = 80.0f;              // K_fast: p_cl ~= 0.974
  //
  // Section 2(c) Estimator + Controller: slow estimator pole (both ON) 
  // For 2(c), use K_nom and L_slow (10x slower than controller):
  //   kPosGain = 40.0f;
  setEstimatorGain(-0.00132f);      // L_slow: p_est ~= 0.9987 

  
resetController();
  return true;
}

void Robot::control() {

  float volt_A = 0.0f;
  float volt_B = 0.0f;
  Matrix<1> desired_velocity;           
  desired_velocity.Fill(0);

  const float speedA = getSpeedMotorA();
  const float speedB = getSpeedMotorB();
  const float avgSpeed = 0.5f * (speedA + speedB);
  const float frontDistance = getFrontDistance(); 

  if(StateEstimationEnabled()) {   
    
    Matrix<1> distance_measurement;                  
    distance_measurement(0) = frontDistance;         
    CorrectionUpdate(distance_measurement, _xhat, _nu);   
  }
  if(controlEnabled()) {   

    xref(0) = -desiredDistanceMeters;

    K(0) = kPosGain;

    float feedbackPos;
    if (StateEstimationEnabled()) {
      feedbackPos = _xhat(0);          
    } else {
      feedbackPos = -frontDistance;      
    }
    const float positionError = xref(0) - feedbackPos;
    float v_ref = kPosGain * positionError;
    v_ref = saturate(v_ref, kVelRefLimit); 
    desired_velocity(0) = v_ref;

    const float eA = v_ref - speedA;
    const float eB = v_ref - speedB;
    const float uA = applyPi(eA, piStateA, coeffA);
    const float uB = applyPi(eB, piStateB, coeffB);

    volt_A = saturate(uA, kVoltageLimit);
    volt_B = saturate(uB, kVoltageLimit);

    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  }
  else                      
  {
    desired_velocity(0) = 0.0;
    setVoltageMotorA(0.0);
    setVoltageMotorB(0.0);
    resetController();
  }

  if(StateEstimationEnabled()) {   
    Matrix<1> velocity_input;
    velocity_input(0) = avgSpeed; 
    PredictionUpdate(velocity_input, _xhat);                    
  }
  
  writeValue(1, frontDistance);
  writeValue(2, _xhat(0));
  writeValue(3, desired_velocity(0));
  writeValue(4, speedA);
  writeValue(5, speedB);
  writeValue(6, volt_A);
  writeValue(7, volt_B);
  writeValue(8, _nu(0));
  writeValue(9, xref(0));
  writeValue(12, avgSpeed);
  writeValue(13, kPosGain);
  writeValue(14, getEstimatorGain());


}

void Robot::resetController(){
  piStateA.errorPrev = 0.0f;
  piStateA.controlPrev = 0.0f;
  piStateB.errorPrev = 0.0f;
  piStateB.controlPrev = 0.0f;
  setVoltageMotorA(0.0f);
  setVoltageMotorB(0.0f);
}

void Robot::resetStateEstimator() {
  _nu.Fill(0);
  
  constexpr float kWrongInitialEstimate = -0.00f;
  _xhat(0) = kWrongInitialEstimate;
}

float Robot::applyPi(float error, PiState &state, const PiCoeffs &coeffs) const {
  const float control = coeffs.b0 * error + coeffs.b1 * state.errorPrev +
                        coeffs.feedback * state.controlPrev;
  state.errorPrev = error;
  state.controlPrev = control;
  return control;
}

float Robot::saturate(float value, float limit) const {
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

bool Robot::controlEnabled() {
  return _button_states[0];       
}

bool Robot::StateEstimationEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  if(toggleButton(0)) {           
    _button_states[1] = 1;        
    resetController();
    resetStateEstimator();
    desiredDistanceMeters = 0.30f;
    message("Controller + estimator enabled (Button 0).");    
  }
  else {
    _button_states[1] = 0;
    message("Controller + estimator disabled.");
  }
}

void Robot::button1callback() {
  
  desiredDistanceMeters = 0.30f;
  message("Desired distance set to 0.30 m (Button 1).");
}

void Robot::button2callback() {
    init();                         
    resetController();              
    _button_states[0] = 0;
    resetStateEstimator();            
    _button_states[1] = 0;
    message("MECOtron reinitialized.");
}
