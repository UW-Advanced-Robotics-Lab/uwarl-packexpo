/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_InverseKinematicsWAM_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 07-Sep-2019 20:43:14
 */

#ifndef _CODER_INVERSEKINEMATICSWAM_API_H
#define _CODER_INVERSEKINEMATICSWAM_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_InverseKinematicsWAM_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void InverseKinematicsWAM(real_T toolPoseDes[7], real_T JnCur[7], real_T
  Theta_Jn[7], real_T *success);
extern void InverseKinematicsWAM_api(const mxArray * const prhs[2], int32_T nlhs,
  const mxArray *plhs[2]);
extern void InverseKinematicsWAM_atexit(void);
extern void InverseKinematicsWAM_initialize(void);
extern void InverseKinematicsWAM_terminate(void);
extern void InverseKinematicsWAM_xil_terminate(void);

#endif

/*
 * File trailer for _coder_InverseKinematicsWAM_api.h
 *
 * [EOF]
 */
