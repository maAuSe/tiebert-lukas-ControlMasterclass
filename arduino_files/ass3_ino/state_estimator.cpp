#include "state_estimator.h"

namespace {
constexpr float kTs = 0.01f;           // s
constexpr float kWheelRadius = 0.033f; // m
constexpr float kC = -1.0f;            // y = -x
float g_estimator_gain = -0.18f;       // default observer gain (radial pole at 1+L)
}

void setEstimatorGain(float gain) { g_estimator_gain = gain; }
float getEstimatorGain() { return g_estimator_gain; }

void PredictionUpdate(const Matrix<1> &u, Matrix<1> &xhat) {
  // Discrete-time A and B matrices for x[k+1] = A x[k] + B u[k]
  float arrayA[1][1]{{1.0f}};
  Matrix<1,1> A = arrayA;
  float arrayB[1][1]{{kTs * kWheelRadius}};
  Matrix<1,1> B = arrayB;

  // A priori update
  xhat = A * xhat + B * u;
}

void CorrectionUpdate(const Matrix<1> &y, Matrix<1> &xhat, Matrix<1> &nu) {
  float arrayC[1][1]{{kC}};
  Matrix<1,1> C = arrayC;

  // Innovation
  nu = y - C * xhat;

  // Estimator gain (L is scalar)
  Matrix<1,1> L;
  L(0,0) = g_estimator_gain;

  // A posteriori update
  xhat += L * nu;
}
