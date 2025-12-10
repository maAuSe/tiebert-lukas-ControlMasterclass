#include "extended_kalman_filter.h"
#include <math.h>

namespace {
constexpr float kTs = TSAMPLE;

// Geometry (measure on the platform; default values are placeholders in meters)
constexpr float kAlpha = 0.090f;  // front sensor offset along X' from cart center
constexpr float kBeta  = 0.085f;  // longitudinal offset of side sensor from cart center
constexpr float kGamma = 0.060f;  // lateral offset of side sensor from cart center (positive to the left)

// Walls: p*x + q*y = r  (default: W1 is y=0, W2 is x=0, robot starts in x<0, y<0 quadrant)
constexpr float kP1 = 0.0f; constexpr float kQ1 = 1.0f; constexpr float kR1 = 0.0f;  // W1: y = 0
constexpr float kP2 = 1.0f; constexpr float kQ2 = 0.0f; constexpr float kR2 = 0.0f;  // W2: x = 0

// Noise covariances (tune experimentally; units: m^2 for position, rad^2 for heading)
constexpr float kQx = 1.0e-5f;
constexpr float kQy = 1.0e-5f;
constexpr float kQtheta = 5.0e-6f;
constexpr float kRz1 = 1.0e-4f;
constexpr float kRz2 = 1.0e-4f;

inline float wallNorm(float p, float q) {
  return sqrtf(p * p + q * q);
}
}

void PredictionUpdate(const Matrix<2> &u, Matrix<3> &xhat, Matrix<3,3> &Phat) {
  const float v = u(0);
  const float omega = u(1);
  const float theta = xhat(2);

  const float cth = cosf(theta);
  const float sth = sinf(theta);

  // Nonlinear state update (forward Euler discretization)
  xhat(0) += kTs * v * cth;
  xhat(1) += kTs * v * sth;
  xhat(2) += kTs * omega;

  // Jacobian of the discretized dynamics: A = I + Ts * df/dx
  float arrayA[3][3] = {
    {1.0f, 0.0f, -kTs * v * sth},
    {0.0f, 1.0f,  kTs * v * cth},
    {0.0f, 0.0f,  1.0f}
  };
  Matrix<3, 3> A = arrayA;

  float arrayQ[3][3]{ { kQx,   0.0f,    0.0f},
                      { 0.0f,  kQy,     0.0f},
                      { 0.0f,  0.0f, kQtheta}};
  Matrix<3, 3> Q = arrayQ;

  // Covariance propagation
  Phat = A * Phat * A.Transpose() + Q;
}

void CorrectionUpdate(const Matrix<2> &y, Matrix<3> &xhat, Matrix<3,3> &Phat, Matrix<2> &nu, Matrix<2,2> &S) {
  const float theta = xhat(2);
  const float cth = cosf(theta);
  const float sth = sinf(theta);

  // Sensor positions in world frame
  const float xf = xhat(0) + kAlpha * cth;
  const float yf = xhat(1) + kAlpha * sth;
  const float xs = xhat(0) + kBeta * cth - kGamma * sth;
  const float ys = xhat(1) + kBeta * sth + kGamma * cth;

  const float n1 = wallNorm(kP1, kQ1);
  const float n2 = wallNorm(kP2, kQ2);

  // Measurement predictions
  Matrix<2> h;
  h(0) = (kR1 - kP1 * xf - kQ1 * yf) / n1;  // front IR distance to W1
  h(1) = (kR2 - kP2 * xs - kQ2 * ys) / n2;  // lateral IR distance to W2

  // Jacobian of h with respect to states
  const float dx1dtheta = -kAlpha * sth;
  const float dy1dtheta =  kAlpha * cth;
  const float dx2dtheta = -kBeta * sth - kGamma * cth;
  const float dy2dtheta =  kBeta * cth - kGamma * sth;

  float arrayJh[2][3] = {
    {-kP1 / n1, -kQ1 / n1, -(kP1 * dx1dtheta + kQ1 * dy1dtheta) / n1},
    {-kP2 / n2, -kQ2 / n2, -(kP2 * dx2dtheta + kQ2 * dy2dtheta) / n2}
  };
  Matrix<2,3> C = arrayJh;

  // Measurement noise
  float arrayR[2][2]{{kRz1, 0.0f},
                     {0.0f, kRz2}};
  Matrix<2, 2> R = arrayR;

  // Innovation and innovation covariance
  nu = y - h;
  S = C * Phat * C.Transpose() + R;

  // Kalman gain
  Matrix<3,2> L = Phat * C.Transpose() * S.Inverse();

  // State and covariance correction
  xhat += L * nu;
  Phat -= L * C * Phat;
}
