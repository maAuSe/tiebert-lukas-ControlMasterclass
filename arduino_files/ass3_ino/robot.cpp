/*
 * STATE ESTIMATION AND STATE FEEDBACK CONTROL TEMPLATE
 *
 * This is a template to get you started with the implementation of the state estimation and state feedback control
 * on your own cart.
 *
 */

#include "robot.h"

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  xref.Fill(0);
  _xhat.Fill(0);
  _nu.Fill(0);
  setEstimatorGain(-0.18f);
  resetController();
  return true;
}

void Robot::control() {

  float volt_A = 0.0f;
  float volt_B = 0.0f;
  Matrix<1> desired_velocity;           // control signal (rad/s)
  desired_velocity.Fill(0);

  const float speedA = getSpeedMotorA();
  const float speedB = getSpeedMotorB();
  const float avgSpeed = 0.5f * (speedA + speedB);
  const float frontDistance = getFrontDistance(); // m, positive

  // State estimation
  if(StateEstimationEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Correction step
    Matrix<1> distance_measurement;                   // define a vector of length 1
    distance_measurement(0) = frontDistance;          // front distance (m)
    CorrectionUpdate(distance_measurement, _xhat, _nu);   // do the correction step -> update _xhat, _nu
  }
  if(controlEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 0' in QRoboticsCenter)

    // Read position reference as distance to wall [m]; convert to cart position [m] (negative in front of wall)
    const float desired_distance = readValue(0); // channel 0 reserved for reference input
    xref(0) = -desired_distance;

    // Optional runtime gain override from QRC (channels 10 for K, 11 for L)
    const float kOverride = readValue(10);
    const float lOverride = readValue(11);
    if(kOverride > 0.0001f || kOverride < -0.0001f) {
      kPosGain = kOverride;
    }
    if(lOverride > 0.0001f || lOverride < -0.0001f) {
      setEstimatorGain(lOverride);
    }
    K(0) = kPosGain;

    // State feedback to desired velocity (rad/s)
    // Use estimator output if enabled, else use direct measurement (spec 2b)
    float feedbackPos;
    if (StateEstimationEnabled()) {
      feedbackPos = _xhat(0);            // use estimator output
    } else {
      feedbackPos = -frontDistance;      // use direct measurement (x = -distance)
    }
    const float positionError = xref(0) - feedbackPos;
    float v_ref = kPosGain * positionError;
    v_ref = saturate(v_ref, kVelRefLimit); // avoid excessive speed commands
    desired_velocity(0) = v_ref;

    // Velocity controller (PI from Assignment 2)
    const float eA = v_ref - speedA;
    const float eB = v_ref - speedB;
    const float uA = applyPi(eA, piStateA, coeffA);
    const float uB = applyPi(eB, piStateB, coeffB);

    volt_A = saturate(uA, kVoltageLimit);
    volt_B = saturate(uB, kVoltageLimit);

    // Send wheel speed command
    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  }
  else                      // do nothing since control is disabled
  {
    desired_velocity(0) = 0.0;
    setVoltageMotorA(0.0);
    setVoltageMotorB(0.0);
    resetController();
  }

  // Sate estimation
  if(StateEstimationEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Prediction step
    Matrix<1> velocity_input;
    velocity_input(0) = avgSpeed; // use measured speed for prediction
    PredictionUpdate(velocity_input, _xhat);                    // do the prediction step -> update _xhat
  }
  // writeValue(8, _xhat(0)); // a priori state estimate
  
  // Send useful outputs to QRC
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
  // Reset PI states and set voltages to zero
  piStateA.errorPrev = 0.0f;
  piStateA.controlPrev = 0.0f;
  piStateB.errorPrev = 0.0f;
  piStateB.controlPrev = 0.0f;
  setVoltageMotorA(0.0f);
  setVoltageMotorB(0.0f);
}

void Robot::resetStateEstimator() {
  _nu.Fill(0);
  float initGuess = readValue(5); // optional: set wrong initial estimate from QRC
  if(initGuess < 0.0001f && initGuess > -0.0001f) {
    initGuess = kDefaultX0;
  }
  _xhat(0) = initGuess;
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
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::StateEstimationEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    resetController();
    message("Controller reset and enabled.");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    message("Control disabled.");
  }
  // button1callback();           // If you want to toggle the estimator together with the controller
}

void Robot::button1callback() {
  if(toggleButton(1)){
      resetStateEstimator();            // Reset the state estimator
      message("State estimator reset and enabled.");
  }
  else
  {
    message("State estimator disabled.");
  }
}

void Robot::button2callback() {
    init();                         // Reset the MECOtron and reinitialize the Robot object
    resetController();              // Reset the Controller
    _button_states[0] = 0;
    resetStateEstimator();            // Reset the state estimator
    _button_states[1] = 0;
    message("MECOtron reinitialized.");
}
