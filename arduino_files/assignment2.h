#ifndef ASSIGNMENT2_H
#define ASSIGNMENT2_H

#include "mecotron.h"

class Robot : public MECOtron {
 public:
  Robot();

  bool init();
  void control();
  bool controlEnabled();

  void button0callback();
  void button1callback();

 private:
  enum ControllerMode : uint8_t {
    MODE_NOMINAL = 0,
    MODE_LOW_BAND = 1
  };

  struct PiCoeffs {
    float b0;
    float b1;
    float feedback;
  };

  struct PiState {
    float errorPrev;
    float controlPrev;
  };

  static constexpr float kVoltageLimit = 11.0f;
  static constexpr float kReferenceLimit = 15.0f;

  ControllerMode activeMode;
  PiState stateA;
  PiState stateB;
  PiCoeffs coeffsA[2];
  PiCoeffs coeffsB[2];

  float saturate(float value, float limit) const;
  float applyPi(float error, PiState &state, const PiCoeffs &coeffs) const;
  void resetControllerStates();
  void streamTelemetry(float reference, float errorA, float errorB,
                       float controlA, float controlB,
                       float speedA, float speedB) const;
};

#endif /* ASSIGNMENT2_H */
