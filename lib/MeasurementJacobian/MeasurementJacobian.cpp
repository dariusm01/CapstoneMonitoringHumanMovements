//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: MeasurementJacobian.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:16:36
//

// Include Files
#include "MeasurementJacobian.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double phi
//                double theta
//                double H[9]
// Return Type  : void
//
void MeasurementJacobian(double phi, double theta, double H[9])
{
  double H_tmp;
  double b_H_tmp;
  double c_H_tmp;
  H_tmp = std::cos(theta);
  b_H_tmp = std::sin(phi);
  c_H_tmp = std::cos(phi);
  H[0] = 0.0;
  H[3] = -H_tmp;
  H[6] = 0.0;
  H[1] = H_tmp * c_H_tmp;
  H_tmp = -std::sin(theta);
  H[4] = H_tmp * b_H_tmp;
  H[7] = 0.0;
  H[2] = -std::cos(theta) * b_H_tmp;
  H[5] = H_tmp * c_H_tmp;
  H[8] = 0.0;
}

//
// File trailer for MeasurementJacobian.cpp
//
// [EOF]
//
