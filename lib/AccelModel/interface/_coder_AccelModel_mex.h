//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_AccelModel_mex.h
//
// MATLAB Coder version            : 5.2
// C/C++ source code generated on  : 04-Apr-2021 11:01:17
//

#ifndef _CODER_ACCELMODEL_MEX_H
#define _CODER_ACCELMODEL_MEX_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_AccelModel_mexFunction(int32_T nlhs, mxArray *plhs[3], int32_T nrhs,
                                   const mxArray *prhs[2]);

#endif
//
// File trailer for _coder_AccelModel_mex.h
//
// [EOF]
//
