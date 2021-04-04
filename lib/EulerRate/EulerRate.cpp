//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: EulerRate.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:11:06
//

// Include Files
#include "EulerRate.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double phi
//                double theta
//                const double Gyro[3]
//                double *phiDot
//                double *thetaDot
//                double *psiDot
// Return Type  : void
//
void EulerRate(double phi, double theta, const double Gyro[3], double *phiDot,
               double *thetaDot, double *psiDot)
{
  double dv[9];
  double EulerRates[3];
  double EulerKinematic_tmp;
  double b_EulerKinematic_tmp;
  double c_EulerKinematic_tmp;
  double d_EulerKinematic_tmp;
  EulerKinematic_tmp = std::tan(theta);
  b_EulerKinematic_tmp = std::sin(phi);
  c_EulerKinematic_tmp = std::cos(phi);
  d_EulerKinematic_tmp = 1.0 / std::cos(theta);
  dv[0] = 1.0;
  dv[3] = b_EulerKinematic_tmp * EulerKinematic_tmp;
  dv[6] = c_EulerKinematic_tmp * EulerKinematic_tmp;
  dv[1] = 0.0;
  dv[4] = c_EulerKinematic_tmp;
  dv[7] = -b_EulerKinematic_tmp;
  dv[2] = 0.0;
  dv[5] = b_EulerKinematic_tmp * d_EulerKinematic_tmp;
  dv[8] = c_EulerKinematic_tmp * d_EulerKinematic_tmp;
  EulerKinematic_tmp = Gyro[0];
  b_EulerKinematic_tmp = Gyro[1];
  c_EulerKinematic_tmp = Gyro[2];
  for (int i{0}; i < 3; i++) {
    EulerRates[i] =
        (dv[i] * EulerKinematic_tmp + dv[i + 3] * b_EulerKinematic_tmp) +
        dv[i + 6] * c_EulerKinematic_tmp;
  }
  *phiDot = EulerRates[0];
  *thetaDot = EulerRates[1];
  *psiDot = EulerRates[2];
}

//
// File trailer for EulerRate.cpp
//
// [EOF]
//
