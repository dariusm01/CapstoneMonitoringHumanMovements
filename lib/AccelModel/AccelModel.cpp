//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: AccelModel.cpp
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:01:17
//

// Include Files
#include "AccelModel.h"
#include <cmath>

// Function Definitions
//
// Arguments    : double Phi
//                double Theta
//                double *ax
//                double *ay
//                double *az
// Return Type  : void
//
void AccelModel(double Phi, double Theta, double *ax, double *ay, double *az)
{
  double ay_tmp;
  *ax = -std::sin(Theta);
  ay_tmp = std::cos(Theta);
  *ay = ay_tmp * std::sin(Phi);
  *az = ay_tmp * std::cos(Phi);
}

//
// File trailer for AccelModel.cpp
//
// [EOF]
//
