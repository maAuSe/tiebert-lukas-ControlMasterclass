#include "extended_kalman_filter.h"

void PredictionUpdate(const Matrix<2> &u, Matrix<3> &xhat, Matrix<3,3> &Phat) {
  // // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT PredictionUpdate OF THE EXTENDED KALMAN FILTER
  // // Tuning parameter
  // float arrayQ[3][3]{ { ?,  0, 0},    //Provide here the element values of weight Q
  //                     { 0,  ?, 0},
  //                     { 0,  0, ?}};
  //
  // Matrix<3, 3> Q = arrayQ;
  //
  // // Compute Jacobian of system dynamics
  // float arrayJf[3][3]{{?, ?, ?},   //Provide here the element values of the Jacobian of system dynamics
  //                     {?, ?, ?},
  //                     {?, ?, ?}};
  // Matrix<3, 3> A = arrayJf;
  //
  // // Evaluate discrete-time nonlinear system dynamics
  // float arrayf[3][1]{{ ? }, //Provide the nonlinear dynamics equation for each state
  //                    { ? },
  //                    { ? }};
  // xhat = arrayf;    //state prediction is equal to the nonlinear dynamics calculated in arrayf
  //
  // // Update state covariance: P = APAt + Q, with A equal to the Jacobian of system dynamics
  // Phat = A * Phat * A.Transpose() + Q;
}

void CorrectionUpdate(const Matrix<2> &y, Matrix<3> &xhat, Matrix<3,3> &Phat, Matrix<2> &nu, Matrix<2,2> &S) {
  // // UNCOMMENT AND COMPLETE LINES BELOW TO IMPLEMENT CorrectionUpdate OF THE EXTENDED KALMAN FILTER
  // // Define useful constant
  // const float alpha = ? ;  // longitudinal distance from frontal IR sensor to front wheel wheel axle
  // const float beta = ? ;   // longitudinal distance from lateral IR sensor to front wheel axle
  // const float gamma = ? ;  // lateral distance from lateral IR sensor to middle of front wheel axle
  //
  // // Tuning parameter
  // float arrayR[2][2]{{?, 0},    //Provide here the element values of weight R
  //                    {0, ?}};
  // Matrix<2, 2> R = arrayR;
  //
  // // System C-matrix - Compute Jacobian of measurement equation
  // float arrayJh[2][3]{{?, ?, ?}, //Provide here the element values of state-space matrix C
  //                     {?, ?, ?}};
  // Matrix<2,3> C = arrayJh;
  //
  // // Evaluate measurement equation
  // float arrayh[2][1]{{ ? },
  //                    { ? }};
  // Matrix<2> h = arrayh;
  //
  // // Compute innovation
  // nu = y - h;
  //
  // // Compute innovation covariance
  // S = C * Phat * C.Transpose() + R;
  //
  // // Compute optimal Kalman filter gain
  // Matrix<3,2> L = Phat * C.Transpose() * S.Inverse();
  //
  // // Compute corrected system state estimate
  // xhat += L * nu;
  //
  // // Compute corrected state estimate covariance
  // Phat -= L * C * Phat;
}
