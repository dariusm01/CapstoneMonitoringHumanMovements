//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_EulerRate_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:11:06
//

#ifndef _CODER_EULERRATE_API_H
#define _CODER_EULERRATE_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void EulerRate(real_T phi, real_T theta, real_T Gyro[3], real_T *phiDot,
               real_T *thetaDot, real_T *psiDot);

void EulerRate_api(const mxArray *const prhs[3], int32_T nlhs,
                   const mxArray *plhs[3]);

void EulerRate_atexit();

void EulerRate_initialize();

void EulerRate_terminate();

void EulerRate_xil_shutdown();

void EulerRate_xil_terminate();

#endif
//
// File trailer for _coder_EulerRate_api.h
//
// [EOF]
//
