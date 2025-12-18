/*
 * EXTENDED KALMAN FILTER TEMPLATE
 *
 * This is a template to get you started with the implementation of the Kalman filter
 * on your own cart.
 *
 */

#include "robot.h"
#include <math.h>

namespace {
constexpr float kWheelRadius = R_WHEEL;
constexpr float kWheelbase = WHEELBASE;
}

bool Robot::init() {
  MECOtron::init(); // Initialize the MECOtron

  desiredVelocityCart.Fill(0);
  _xhat.Fill(0);
  _Phat.Fill(0);
  _nu.Fill(0);
  _S.Fill(0);
  _errorBody.Fill(0);

  // PI velocity controllers (nominal bandwidth from Assignment 2)
  piCoeffsA = {0.945014f, -0.804389f, 1.0f};
  piCoeffsB = {0.919504f, -0.782675f, 1.0f};
  resetVelocityController();

  resetKalmanFilter();
  resetLqrController();
  return true;
}

void Robot::control() {

  float volt_A = 0.0f;
  float volt_B = 0.0f;
  Matrix<2> measurements; measurements.Fill(0);
  const bool hasMeas = trajectory.hasMeasurements();
  const bool trajectoryEnabled = _button_states[2];

  const float speedA = getSpeedMotorA();
  const float speedB = getSpeedMotorB();

  // Correction update (only when measurements are meaningful)
  if(hasMeas){
    measurements(0) = getFrontDistance();
    measurements(1) = getSideDistance();
    if(KalmanFilterEnabled()) {
      CorrectionUpdate(measurements, _xhat, _Phat, _nu, _S);
    } else {
      _nu.Fill(0);
      _S.Fill(0);
    }
  } else {
    measurements(0) = NAN;
    measurements(1) = NAN;
    _nu.Fill(0);
    _S.Fill(0);
  }

  // Reference trajectory
  const float x_ref     = trajectory.X();
  const float y_ref     = trajectory.Y();
  const float theta_ref = trajectory.Theta();

  Matrix<2> uApplied;
  uApplied.Fill(0);
  if(trajectoryEnabled) {
    // Compute body-frame tracking error for LQR 
    computeBodyFrameError(x_ref, y_ref, theta_ref, _xhat(0), _xhat(1), _xhat(2), _errorBody);

    // LQR state feedback: u = K * e'
    uApplied = _Klqr * _errorBody;
  } else {
    _errorBody.Fill(0);
  }

  if(controlEnabled()) {
    // Map cart velocities to wheel angular velocities (rad/s)
    const float desiredVelocityMotorA = (uApplied(0) - 0.5f * kWheelbase * uApplied(1)) / kWheelRadius;
    const float desiredVelocityMotorB = (uApplied(0) + 0.5f * kWheelbase * uApplied(1)) / kWheelRadius;

    const float eA = saturate(desiredVelocityMotorA, kVelRefLimit) - speedA;
    const float eB = saturate(desiredVelocityMotorB, kVelRefLimit) - speedB;

    volt_A = saturate(applyPi(eA, piStateA, piCoeffsA), kVoltageLimit);
    volt_B = saturate(applyPi(eB, piStateB, piCoeffsB), kVoltageLimit);

    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  } else {
    uApplied.Fill(0);
    resetVelocityController();
    setVoltageMotorA(0.0f);
    setVoltageMotorB(0.0f);
  }

  // Store the input used for prediction (zero when control is disabled)
  desiredVelocityCart = uApplied;

  if(KalmanFilterEnabled()) {
    PredictionUpdate(desiredVelocityCart, _xhat, _Phat);
  }

  // Send outputs to QRC (channels 0-11 for MATLAB postprocessing)
  writeValue(0, x_ref);            // reference x
  writeValue(1, y_ref);            // reference y
  writeValue(2, theta_ref);        // reference theta
  writeValue(3, _xhat(0));         // EKF estimated x
  writeValue(4, _xhat(1));         // EKF estimated y
  writeValue(5, _xhat(2));         // EKF estimated theta
  writeValue(6, _errorBody(0));    // body-frame error e'_x
  writeValue(7, _errorBody(1));    // body-frame error e'_y
  writeValue(8, _errorBody(2));    // body-frame error e'_theta
  writeValue(9, uApplied(0));      // applied v (LQR feedback)
  writeValue(10, uApplied(1));     // applied omega (LQR feedback)
  writeValue(11, measurements(0)); // z1 (front IR, for debugging)

  //triggers the trajectory to return the next values during the next cycle
  trajectory.update();
}

void Robot::resetController(){
  desiredVelocityCart.Fill(0);
  _errorBody.Fill(0);
  resetVelocityController();
  setVoltageMotorA(0.0f);
  setVoltageMotorB(0.0f);
}

void Robot::resetLqrController() {
  // LQR gain matrix K (2x3) from dlqr(Ad, Bd, Q_lqr, R_lqr)
  // Computed in MATLAB: Q_lqr = diag([16, 16, 4]), R_lqr = diag([0.1, 0.1])
  // IMPORTANT: dlqr returns K for u = -K*x, but we use u = K*e', so NEGATE!
  // K_firmware = -K_dlqr
  // Aggressive tuning: ~4x gains for faster tracking
  _Klqr = {29.9450f, -0.0000f, -0.0000f,
           -0.0000f, -32.7134f, 16.3999f};
  _errorBody.Fill(0);
}

void Robot::computeBodyFrameError(float x_ref, float y_ref, float theta_ref,
                                   float x_hat, float y_hat, float theta_hat,
                                   Matrix<3> &e_body) {
  // World-frame tracking error: e = xi_ref - xi_hat
  const float ex_world = x_ref - x_hat;
  const float ey_world = y_ref - y_hat;
  float etheta = theta_ref - theta_hat;

  // Wrap angle error to [-pi, pi]
  while(etheta >  3.14159265f) etheta -= 6.28318531f;
  while(etheta < -3.14159265f) etheta += 6.28318531f;

  // Rotation matrix R(theta_hat) to convert world -> body frame (spec 4a)
  // e' = R(theta_hat) * e_world
  // R = [ cos(th)  sin(th)  0 ]
  //     [-sin(th)  cos(th)  0 ]
  //     [   0        0      1 ]
  const float cth = cosf(theta_hat);
  const float sth = sinf(theta_hat);

  e_body(0) =  cth * ex_world + sth * ey_world;  // e'_x (longitudinal error)
  e_body(1) = -sth * ex_world + cth * ey_world;  // e'_y (lateral error)
  e_body(2) = etheta;                            // e'_theta (heading error)
}

void Robot::resetKalmanFilter() {
  _Phat.Fill(0);      // Initialize the covariance matrix
  // Initial covariance (reflects a few cm and ~5 deg uncertainty)
  _Phat(0,0) = 1.0e-3f;     // (m^2) -> std ≈ 3.2 cm
  _Phat(1,1) = 1.0e-3f;     // (m^2) -> std ≈ 3.2 cm
  _Phat(2,2) = 7.6e-3f;     // (rad^2) -> std ≈ 5 deg

  // Initialize state estimate near starting pose (-0.30, -0.20) m, heading aligned with +X
  _xhat(0) = -0.30f;
  _xhat(1) = -0.20f;
  _xhat(2) = 0.0f;

  // Reset innovation and its covariance matrix
  _S.Fill(0);
  _nu.Fill(0);
}

bool Robot::controlEnabled() {
  return _button_states[0];       // The control is enabled if the state of button 0 is true
}

bool Robot::KalmanFilterEnabled() {
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
}

void Robot::button1callback() {
  if(toggleButton(1)){
      resetKalmanFilter();            // Reset the Kalman filter
      message("Kalman filter reset and enabled.");
  }
  else
  {
    message("Kalman filter disabled.");
  }
}

void Robot::button2callback() {
  if(toggleButton(2)) {
    trajectory.start();
    message("Trajectory started/resumed.");
  } else {
    trajectory.stop();
    message("Trajectory stopped.");
  }
}

void Robot::button3callback() {
    _button_states[2] = 0;
    trajectory.reset();
    message("Trajectory reset.");
}

void Robot::resetVelocityController() {
  piStateA.errorPrev = 0.0f;
  piStateA.controlPrev = 0.0f;
  piStateB.errorPrev = 0.0f;
  piStateB.controlPrev = 0.0f;
}

float Robot::saturate(float value, float limit) const {
  if(value > limit) return limit;
  if(value < -limit) return -limit;
  return value;
}

float Robot::applyPi(float error, PiState &state, const PiCoeffs &coeffs) const {
  const float control = coeffs.b0 * error + coeffs.b1 * state.errorPrev +
                        coeffs.feedback * state.controlPrev;
  state.errorPrev = error;
  state.controlPrev = control;
  return control;
}
