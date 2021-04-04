//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_MeasurementJacobian_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:16:36
//

#ifndef _CODER_MEASUREMENTJACOBIAN_API_H
#define _CODER_MEASUREMENTJACOBIAN_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void MeasurementJacobian(real_T phi, real_T theta, real_T H[9]);

void MeasurementJacobian_api(const mxArray *const prhs[2],
                             const mxArray **plhs);

void MeasurementJacobian_atexit();

void MeasurementJacobian_initialize();

void MeasurementJacobian_terminate();

void MeasurementJacobian_xil_shutdown();

void MeasurementJacobian_xil_terminate();

#endif
//
// File trailer for _coder_MeasurementJacobian_api.h
//
// [EOF]
//
