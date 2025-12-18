#include "extended_kalman_filter.h"
#include <math.h>

namespace {
constexpr float kTs = TSAMPLE;

// Geometry (measure on the platform; values in meters)
constexpr float kAlpha = 0.075f;  // front sensor offset along X' from cart center
constexpr float kBeta  = 0.065f;  // longitudinal offset of side sensor from cart center
constexpr float kGamma = 0.078f;  // lateral offset of side sensor from cart center (positive to the left)

// Scalars to quickly sweep Q/R ratios for spec 3(b)
// Keep these at 1 and apply tuning directly on Q/R entries below.
constexpr float kQScale = 1.0f;
constexpr float kRScale = 5.0f;

// Walls: p*x + q*y = r  (default: W1 is y=0, W2 is x=0, robot starts in x<0, y<0 quadrant)
constexpr float kP1 = 0.0f; constexpr float kQ1 = 1.0f; constexpr float kR1 = 0.0f;  // W1: y = 0
constexpr float kP2 = 1.0f; constexpr float kQ2 = 0.0f; constexpr float kR2 = 0.0f;  // W2: x = 0

// Noise covariances (tune experimentally; units: m^2 for position, rad^2 for heading)
// Tuned values from assignment5.tex eq. (14)-(15)
constexpr float kQx = kQScale * (8.0e-9f);      // 8e-9 m^2
constexpr float kQy = kQScale * (9.0e-8f);      // 9e-8 m^2
constexpr float kQtheta = kQScale * (9.0e-7f);  // 9e-7 rad^2

// Measurement noise from assignment5.tex eq. (15)
constexpr float kRz1 = kRScale * (0.0198f);  // 0.0198 m^2 (front sensor)
constexpr float kRz2 = kRScale * (0.09f);    // 0.09 m^2 (side sensor)

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
  Matrix<3, 3> A = {1.0f, 0.0f, -kTs * v * sth,
                    0.0f, 1.0f,  kTs * v * cth,
                    0.0f, 0.0f,  1.0f};

  Matrix<3, 3> Q = {kQx,   0.0f,    0.0f,
                    0.0f,  kQy,     0.0f,
                    0.0f,  0.0f, kQtheta};

  // Covariance propagation
  Phat = A * Phat * ~A + Q;
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

  Matrix<2,3> C = {-kP1 / n1, -kQ1 / n1, -(kP1 * dx1dtheta + kQ1 * dy1dtheta) / n1,
                   -kP2 / n2, -kQ2 / n2, -(kP2 * dx2dtheta + kQ2 * dy2dtheta) / n2};

  // Measurement noise
  Matrix<2, 2> R = {kRz1, 0.0f,
                    0.0f, kRz2};

  // Innovation and innovation covariance
  nu = y - h;
  S = C * Phat * ~C + R;

  // Kalman gain
  Matrix<2,2> Sinv = S;
  Invert(Sinv);
  Matrix<3,2> L = Phat * ~C * Sinv;

  // State and covariance correction
  xhat += L * nu;
  Phat -= L * C * Phat;
}
