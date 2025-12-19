#include "extended_kalman_filter.h"
#include <math.h>

namespace {
constexpr float kTs = TSAMPLE;

constexpr float kAlpha = 0.075f;  
constexpr float kBeta  = 0.065f;  
constexpr float kGamma = 0.078f;  


constexpr float kQScale = 1.0f;
constexpr float kRScale = 1.0f;


constexpr float kP1 = 1.0f; constexpr float kQ1 = 0.0f; constexpr float kR1 = 0.0f;  
constexpr float kP2 = 0.0f; constexpr float kQ2 = 1.0f; constexpr float kR2 = 0.0f;  


constexpr float kQx = kQScale * (8.0e-9f);      
constexpr float kQy = kQScale * (9.0e-8f);      
constexpr float kQtheta = kQScale * (9.0e-7f);  

constexpr float kRz1 = kRScale * (0.0198f);  
constexpr float kRz2 = kRScale * (0.09f);    

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

  xhat(0) += kTs * v * cth;
  xhat(1) += kTs * v * sth;
  xhat(2) += kTs * omega;

  Matrix<3, 3> A = {1.0f, 0.0f, -kTs * v * sth,
                    0.0f, 1.0f,  kTs * v * cth,
                    0.0f, 0.0f,  1.0f};

  Matrix<3, 3> Q = {kQx,   0.0f,    0.0f,
                    0.0f,  kQy,     0.0f,
                    0.0f,  0.0f, kQtheta};

  Phat = A * Phat * ~A + Q;
}

void CorrectionUpdate(const Matrix<2> &y, Matrix<3> &xhat, Matrix<3,3> &Phat, Matrix<2> &nu, Matrix<2,2> &S) {
  const float theta = xhat(2);
  const float cth = cosf(theta);
  const float sth = sinf(theta);

  const float xf = xhat(0) + kAlpha * cth;
  const float yf = xhat(1) + kAlpha * sth;
  const float xs = xhat(0) + kBeta * cth - kGamma * sth;
  const float ys = xhat(1) + kBeta * sth + kGamma * cth;

  const float n1 = wallNorm(kP1, kQ1);
  const float n2 = wallNorm(kP2, kQ2);

  Matrix<2> h;
  h(0) = (kR1 - kP1 * xf - kQ1 * yf) / n1;  
  h(1) = (kR2 - kP2 * xs - kQ2 * ys) / n2;  

  const float dx1dtheta = -kAlpha * sth;
  const float dy1dtheta =  kAlpha * cth;
  const float dx2dtheta = -kBeta * sth - kGamma * cth;
  const float dy2dtheta =  kBeta * cth - kGamma * sth;

  Matrix<2,3> C = {-kP1 / n1, -kQ1 / n1, -(kP1 * dx1dtheta + kQ1 * dy1dtheta) / n1,
                   -kP2 / n2, -kQ2 / n2, -(kP2 * dx2dtheta + kQ2 * dy2dtheta) / n2};

  Matrix<2, 2> R = {kRz1, 0.0f,
                    0.0f, kRz2};

  nu = y - h;
  S = C * Phat * ~C + R;

  Matrix<2,2> Sinv = S;
  Invert(Sinv);
  Matrix<3,2> L = Phat * ~C * Sinv;

  xhat += L * nu;
  Phat -= L * C * Phat;
}

