/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_InverseKinematicsWAM_api.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 07-Sep-2019 20:43:14
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_InverseKinematicsWAM_api.h"
#include "_coder_InverseKinematicsWAM_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "InverseKinematicsWAM",              /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[7];
static const mxArray *b_emlrt_marshallOut(const real_T u);
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[7];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *toolPoseDes, const char_T *identifier))[7];
static const mxArray *emlrt_marshallOut(const real_T u[7]);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[7]
 */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[7]
{
  real_T (*y)[7];
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
  static const mxArray *b_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[7]
 */
static real_T (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[7]
{
  real_T (*ret)[7];
  static const int32_T dims[1] = { 7 };

  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "double",
    false, 1U, *(int32_T (*)[1])&dims[0]);
  ret = (real_T (*)[7])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *toolPoseDes
 *                const char_T *identifier
 * Return Type  : real_T (*)[7]
 */
  static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *toolPoseDes, const char_T *identifier))[7]
{
  real_T (*y)[7];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(toolPoseDes), &thisId);
  emlrtDestroyArray(&toolPoseDes);
  return y;
}

/*
 * Arguments    : const real_T u[7]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[7])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 0 };

  static const int32_T iv1[1] = { 7 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[1])&iv1[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[2]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void InverseKinematicsWAM_api(const mxArray * const prhs[2], int32_T nlhs, const
  mxArray *plhs[2])
{
  real_T (*Theta_Jn)[7];
  const mxArray *prhs_copy_idx_0;
  real_T (*toolPoseDes)[7];
  real_T (*JnCur)[7];
  real_T success;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  Theta_Jn = (real_T (*)[7])mxMalloc(sizeof(real_T [7]));
  prhs_copy_idx_0 = emlrtProtectR2012b(prhs[0], 0, false, -1);

  /* Marshall function inputs */
  toolPoseDes = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_0), "toolPoseDes");
  JnCur = emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "JnCur");

  /* Invoke the target function */
  InverseKinematicsWAM(*toolPoseDes, *JnCur, *Theta_Jn, &success);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Theta_Jn);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(success);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InverseKinematicsWAM_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  InverseKinematicsWAM_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InverseKinematicsWAM_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void InverseKinematicsWAM_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_InverseKinematicsWAM_api.c
 *
 * [EOF]
 */
