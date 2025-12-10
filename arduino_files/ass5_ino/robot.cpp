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
  xref.Fill(0);
  _xhat.Fill(0);
  _Phat.Fill(0);
  _nu.Fill(0);
  _S.Fill(0);

  // PI velocity controllers (nominal bandwidth from Assignment 2)
  piCoeffsA = {0.945014f, -0.804389f, 1.0f};
  piCoeffsB = {0.919504f, -0.782675f, 1.0f};
  resetVelocityController();

  // Default state-feedback gains (cart frame) - tune via MATLAB dlqr results
  float arrayKfbInit[2][3] = {
    {-0.6f,  0.0f,  0.0f},
    { 0.0f, -0.6f, -0.45f}
  };
  Kfb = arrayKfbInit;

  resetKalmanFilter();
  return true;
}

void Robot::control() {

  float volt_A = 0.0f;
  float volt_B = 0.0f;
  Matrix<2> uff; uff.Fill(0);
  Matrix<2> ufb; ufb.Fill(0);
  Matrix<2> measurements; measurements.Fill(0);

  const float speedA = getSpeedMotorA();
  const float speedB = getSpeedMotorB();

  // Kalman filtering
  if(KalmanFilterEnabled()) {   // only do this if Kalman filter is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)

    // Correction step
    if(trajectory.hasMeasurements()){                                     // perform the correction step if measurement from the sensor are meaningful
      measurements(0) = getFrontDistance();
      measurements(1) = getSideDistance();
      CorrectionUpdate(measurements, _xhat, _Phat, _nu, _S);     // do the correction step -> update _xhat, _Phat, _nu, _S
    }
  }

  if(controlEnabled()) {   // only do this if controller is enabled (triggered by pushing 'Button 0' in QRoboticsCenter)

    // Reference from built-in trajectory
    xref(0)=trajectory.X();        // desired X cart position [m]
    xref(1)=trajectory.Y();        // desired Y cart position [m]
    xref(2)=trajectory.Theta();    // desired cart angle [rad]

    // Feedforward
    uff(0) = trajectory.v();          // desired forward velocity of the cart (in m/s)
    uff(1) = trajectory.omega();      // desired rotational velocity of the cart (in rad/s)

    // Feedback (cart frame)
    Matrix<3> ew = xref - _xhat;      // world-frame error
    const float cth = cosf(_xhat(2));
    const float sth = sinf(_xhat(2));
    float arrayRw2c[3][3]{{ cth,  sth, 0.0f},
                          {-sth,  cth, 0.0f},
                          {0.0f,  0.0f, 1.0f}};
    Matrix<3, 3> Rw2c = arrayRw2c;
    Matrix<3> ec = Rw2c * ew;         // cart-frame error
    ufb = Kfb * ec;                   // state feedback

    desiredVelocityCart = uff + ufb;  // desired cart velocities (forward + rotational)

    // Map cart velocities to wheel angular velocities (rad/s)
    const float v_cmd = desiredVelocityCart(0);
    const float omega_cmd = desiredVelocityCart(1);
    const float desiredVelocityMotorA = (v_cmd - 0.5f * kWheelbase * omega_cmd) / kWheelRadius;
    const float desiredVelocityMotorB = (v_cmd + 0.5f * kWheelbase * omega_cmd) / kWheelRadius;

    const float eA = saturate(desiredVelocityMotorA, kVelRefLimit) - speedA;
    const float eB = saturate(desiredVelocityMotorB, kVelRefLimit) - speedB;

    volt_A = saturate(applyPi(eA, piStateA, piCoeffsA), kVoltageLimit);
    volt_B = saturate(applyPi(eB, piStateB, piCoeffsB), kVoltageLimit);

    // Send wheel speed command
    setVoltageMotorA(volt_A);
    setVoltageMotorB(volt_B);
  }
  else                      // do nothing since control is disables
  {
   desiredVelocityCart.Fill(0);
   setVoltageMotorA(0.0);
   setVoltageMotorB(0.0);
   resetVelocityController();
  }

  // Kalman filtering
  if(KalmanFilterEnabled()) {   // only do this if Kalman filter is enabled (triggered by pushing 'Button 1' in QRoboticsCenter)
    // Prediction step
    PredictionUpdate(desiredVelocityCart, _xhat, _Phat);                        // do the prediction step -> update _xhat and _Phat
  }

  // Send useful outputs to QRC
  // to check functioning of trajectory and feedforward
  writeValue(0, trajectory.v());
  writeValue(1, trajectory.omega());
  writeValue(2, trajectory.X());
  writeValue(3, trajectory.Y());
  writeValue(4, trajectory.Theta());
  writeValue(5, trajectory.hasMeasurements());
  writeValue(6, getSpeedMotorA());
  writeValue(7, getSpeedMotorB());
  writeValue(8, measurements(0));
  writeValue(9, measurements(1));
  writeValue(10, volt_A);
  writeValue(11, volt_B);
  writeValue(12, _xhat(0));
  writeValue(13, _xhat(1));
  writeValue(14, _xhat(2));
  writeValue(15, _nu(0));
  writeValue(16, _nu(1));
  writeValue(17, _Phat(0,0));
  writeValue(18, _Phat(1,1));
  writeValue(19, _Phat(2,2));

  //triggers the trajectory to return the next values during the next cycle
  trajectory.update();
}

void Robot::resetController(){
  desiredVelocityCart.Fill(0);
  xref.Fill(0);
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
    message("Controller resed and enabled.");    // Display a message in the status bar of QRoboticsCenter
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
