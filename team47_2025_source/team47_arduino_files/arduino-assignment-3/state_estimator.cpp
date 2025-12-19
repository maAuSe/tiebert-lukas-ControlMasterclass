#include "state_estimator.h"

namespace {
constexpr float kTs = 0.01f;           // s
constexpr float kWheelRadius = 0.033f; // m
constexpr float kC = -1.0f;            // y = -x
float g_estimator_gain = -0.18f;      
}

void setEstimatorGain(float gain) { g_estimator_gain = gain; }
float getEstimatorGain() { return g_estimator_gain; }

void PredictionUpdate(const Matrix<1> &u, Matrix<1> &xhat) {
  Matrix<1,1> A;
  A(0,0) = 1.0f;
  Matrix<1,1> B;
  B(0,0) = kTs * kWheelRadius;

  xhat = A * xhat + B * u;
}

void CorrectionUpdate(const Matrix<1> &y, Matrix<1> &xhat, Matrix<1> &nu) {
  Matrix<1,1> C;
  C(0,0) = kC;

  nu = y - C * xhat;

  Matrix<1,1> L;
  L(0,0) = g_estimator_gain;

  xhat += L * nu;
}
