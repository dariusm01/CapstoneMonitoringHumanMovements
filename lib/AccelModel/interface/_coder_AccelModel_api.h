//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_AccelModel_api.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:01:17
//

#ifndef _CODER_ACCELMODEL_API_H
#define _CODER_ACCELMODEL_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void AccelModel(real_T Phi, real_T Theta, real_T *ax, real_T *ay, real_T *az);

void AccelModel_api(const mxArray *const prhs[2], int32_T nlhs,
                    const mxArray *plhs[3]);

void AccelModel_atexit();

void AccelModel_initialize();

void AccelModel_terminate();

void AccelModel_xil_shutdown();

void AccelModel_xil_terminate();

#endif
//
// File trailer for _coder_AccelModel_api.h
//
// [EOF]
//
