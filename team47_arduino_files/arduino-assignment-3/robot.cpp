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
  desiredDistanceMeters = 0.40f;  // initial reference: 0.40 m to wall

  // ========== EXPERIMENT GAIN CONFIGURATION ==========
  // Uncomment ONE line per section depending on which experiment you are running.
  //
  // --- 2(a) Estimator-only: L sweep (controller OFF, estimator ON) ---
  // setEstimatorGain(-0.05f);      // L_slow:  p_est ~= 0.95 (estimator-only sweep)
  // setEstimatorGain(-0.18f);      // L_nom:   p_est ~= 0.82
  // setEstimatorGain(-0.35f);      // L_fast:  p_est ~= 0.65
  //
  // --- 2(b) Controller-only: K sweep (controller ON, estimator OFF) ---
  // kPosGain = 20.0f;              // K_slow: p_cl ~= 0.993
  kPosGain = 40.0f;                 // K_nom:  p_cl ~= 0.987  <-- default
  // kPosGain = 80.0f;              // K_fast: p_cl ~= 0.974
  //
  // --- 2(c) Estimator + Controller: slow estimator pole (both ON) ---
  // For 2(c), use K_nom and L_slow (10x slower than controller):
  //   kPosGain = 40.0f;
  setEstimatorGain(-0.00132f);      // L_slow: p_est ~= 0.9987 (10x slower than p_cl with K_nom)
  // =======================================================
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
    // Desired distance is now button-controlled: start at 0.40 m, button 1 sets 0.15 m
    xref(0) = -desiredDistanceMeters;

    // Gains are now hardcoded in init(); runtime override removed for reproducibility.
    // To change K or L, edit the values in init() and re-flash.
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
  // Hardcoded initial estimate (used for estimator-only and combined runs)
  // Set to +0.15 m for upcoming combined experiment
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
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::StateEstimationEnabled() {
  return _button_states[1];
}

void Robot::button0callback() {
  // Toggle controller AND estimator together
  if(toggleButton(0)) {           // Switches the state of button 0 and checks if the new state is true
    _button_states[1] = 1;        // estimator follows controller state
    resetController();
    resetStateEstimator();
    desiredDistanceMeters = 0.30f;
    message("Controller + estimator enabled (Button 0).");    // Display a message in the status bar of QRoboticsCenter
  }
  else {
    _button_states[1] = 0;
    message("Controller + estimator disabled.");
  }
}

void Robot::button1callback() {
  // Update desired reference from 0.15 m to 0.30 m (no estimator reset)
  desiredDistanceMeters = 0.30f;
  message("Desired distance set to 0.30 m (Button 1).");
}

void Robot::button2callback() {
    init();                         // Reset the MECOtron and reinitialize the Robot object
    resetController();              // Reset the Controller
    _button_states[0] = 0;
    resetStateEstimator();            // Reset the state estimator
    _button_states[1] = 0;
    message("MECOtron reinitialized.");
}
