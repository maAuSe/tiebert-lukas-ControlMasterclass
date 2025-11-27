#include "assignment2.h"

Robot::Robot() : activeMode(MODE_NOMINAL) {
  // Nominal controller coefficients (Wheel A/B) from MATLAB script.
  coeffsA[MODE_NOMINAL] = {0.639426f, -0.590012f, 1.0f};
  coeffsB[MODE_NOMINAL] = {0.623161f, -0.575004f, 1.0f};

  // Low-bandwidth (~0.5 Hz crossover) controller coefficients.
  coeffsA[MODE_LOW_BAND] = {0.490693f, -0.486580f, 1.0f};
  coeffsB[MODE_LOW_BAND] = {0.479090f, -0.475074f, 1.0f};

  resetControllerStates();
}

bool Robot::init() {
  MECOtron::init();
  activeMode = MODE_NOMINAL;
  resetControllerStates();
  return true;
}

void Robot::control() {
  const bool enabled = controlEnabled();
  float referenceCmd = 10;

  const float speedA = getSpeedMotorA();
  const float speedB = getSpeedMotorB();
  const float errorA = referenceCmd - speedA;
  const float errorB = referenceCmd - speedB;

  if (!enabled) {
    resetControllerStates();
    setVoltageMotorA(0.0f);
    setVoltageMotorB(0.0f);
    LED1(OFF);
    LED2(ON);
    streamTelemetry(referenceCmd, errorA, errorB, 0.0f, 0.0f,
                    speedA, speedB);
    return;
  }

  LED1(ON);
  LED2(OFF);

  const PiCoeffs &coeffA = coeffsA[activeMode];
  const PiCoeffs &coeffB = coeffsB[activeMode];

  float uA = applyPi(errorA, stateA, coeffA);
  float uB = applyPi(errorB, stateB, coeffB);

  uA = saturate(uA, kVoltageLimit);
  uB = saturate(uB, kVoltageLimit);

  stateA.controlPrev = uA;
  stateB.controlPrev = uB;
  stateA.errorPrev = errorA;
  stateB.errorPrev = errorB;

  setVoltageMotorA(uA);
  setVoltageMotorB(uB);

  streamTelemetry(referenceCmd, errorA, errorB, uA, uB, speedA, speedB);
}

bool Robot::controlEnabled() {
  return _button_states[0];
}

void Robot::button0callback() {
  if (toggleButton(0)) {
    resetControllerStates();
    message("Velocity control enabled.");
  } else {
    message("Velocity control disabled.");
  }
}

void Robot::button1callback() {
  if (toggleButton(1)) {
    activeMode = MODE_LOW_BAND;
    message("Low-bandwidth PI controller selected.");
  } else {
    activeMode = MODE_NOMINAL;
    message("Nominal PI controller selected.");
  }
  resetControllerStates();
}

float Robot::saturate(float value, float limit) const {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

float Robot::applyPi(float error, PiState &state,
                     const PiCoeffs &coeffs) const {
  return coeffs.b0 * error + coeffs.b1 * state.errorPrev +
         coeffs.feedback * state.controlPrev;
}

void Robot::resetControllerStates() {
  stateA.errorPrev = 0.0f;
  stateB.errorPrev = 0.0f;
  stateA.controlPrev = 0.0f;
  stateB.controlPrev = 0.0f;
}

void Robot::streamTelemetry(float reference, float errorA, float errorB,
                            float controlA, float controlB,
                            float speedA, float speedB) const {
  writeValue(0, reference);
  writeValue(1, speedA);
  writeValue(2, speedB);
  writeValue(3, errorA);
  writeValue(4, errorB);
  writeValue(5, controlA);
  writeValue(6, controlB);
  writeValue(7, static_cast<float>(activeMode));
}
