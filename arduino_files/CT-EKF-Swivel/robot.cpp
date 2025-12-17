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

  // PI velocity controllers (nominal bandwidth from Assignment 2)
  piCoeffsA = {0.945014f, -0.804389f, 1.0f};
  piCoeffsB = {0.919504f, -0.782675f, 1.0f};
  resetVelocityController();

  resetKalmanFilter();
  return true;
}

void Robot::control() {

  float volt_A = 0.0f;
  float volt_B = 0.0f;
  Matrix<2> measurements; measurements.Fill(0);
  const bool hasMeas = trajectory.hasMeasurements();

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

  // Feedforward-only trajectory playback for spec 3(b)
  Matrix<2> uApplied;
  uApplied(0) = trajectory.v();      // forward velocity [m/s]
  uApplied(1) = trajectory.omega();  // rotational velocity [rad/s]

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

  // Send useful outputs to QRC
  // to check functioning of trajectory and feedforward
  writeValue(0, trajectory.v());
  writeValue(1, trajectory.omega());
  writeValue(2, measurements(0));
  writeValue(3, measurements(1));
  writeValue(4, _xhat(0));
  writeValue(5, _xhat(1));
  writeValue(6, _xhat(2));
  writeValue(7, _Phat(0,0));
  writeValue(8, _Phat(1,1));
  writeValue(9, _Phat(2,2));
  writeValue(10, uApplied(0));
  writeValue(11, uApplied(1));

  //triggers the trajectory to return the next values during the next cycle
  trajectory.update();
}

void Robot::resetController(){
  desiredVelocityCart.Fill(0);
  resetVelocityController();
  setVoltageMotorA(0.0f);
  setVoltageMotorB(0.0f);
}

void Robot::resetKalmanFilter() {
  _Phat.Fill(0);      // Initialize the covariance matrix
  // Initial covariance (tune experimentally)
  _Phat(0,0) = 0.04f;     // (m^2)
  _Phat(1,1) = 0.04f;     // (m^2)
  _Phat(2,2) = powf(10.0f * (float)M_PI / 180.0f, 2); // (rad^2) ~10 deg

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
