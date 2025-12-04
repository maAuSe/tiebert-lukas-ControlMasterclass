#ifndef SE_SF_H
#define SE_SF_H

#include <BasicLinearAlgebra.h>
#include "mecotron.h" // Include MECOTRON header

using namespace BLA;

void PredictionUpdate(const Matrix<1> &u, Matrix<1> &xhat);
void CorrectionUpdate(const Matrix<1> &y, Matrix<1> &xhat, Matrix<1> &nu);
void setEstimatorGain(float gain);
float getEstimatorGain();

#endif // SE_SF_H
