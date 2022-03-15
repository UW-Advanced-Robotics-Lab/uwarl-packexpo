/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_KnobOperationWAM_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 18-Sep-2019 00:20:57
 */

#ifndef _CODER_KNOBOPERATIONWAM_API_H
#define _CODER_KNOBOPERATIONWAM_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_KnobOperationWAM_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void KnobOperationWAM(real_T Jp_state[7], boolean_T isRevDir, real_T
  PoseKnobWrtWAM[7], real_T Jp_cmd[287], real_T *success);
extern void KnobOperationWAM_api(const mxArray * const prhs[3], int32_T nlhs,
  const mxArray *plhs[2]);
extern void KnobOperationWAM_atexit(void);
extern void KnobOperationWAM_initialize(void);
extern void KnobOperationWAM_terminate(void);
extern void KnobOperationWAM_xil_terminate(void);

#endif

/*
 * File trailer for _coder_KnobOperationWAM_api.h
 *
 * [EOF]
 */
