#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include <BasicLinearAlgebra.h>
#include "mecotron.h"

void PredictionUpdate(const Matrix<1> &u, Matrix<3> &xhat, Matrix<3,3> &Phat);
void CorrectionUpdate(const Matrix<2> &y, Matrix<3> &xhat, Matrix<3,3> &Phat, Matrix<2> &nu, Matrix<2,2> &S);

#endif // LINEAR_KALMAN_FILTER_H
