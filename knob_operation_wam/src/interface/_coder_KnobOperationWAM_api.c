/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_KnobOperationWAM_api.c
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 18-Sep-2019 00:20:57
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_KnobOperationWAM_api.h"
#include "_coder_KnobOperationWAM_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131466U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "KnobOperationWAM",                  /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[7];
static const mxArray *b_emlrt_marshallOut(const real_T u);
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *isRevDir, const char_T *identifier);
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId);
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[7];
static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Jp_state,
  const char_T *identifier))[7];
static const mxArray *emlrt_marshallOut(const real_T u[287]);
static boolean_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);

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
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
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
 *                const mxArray *isRevDir
 *                const char_T *identifier
 * Return Type  : boolean_T
 */
static boolean_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *isRevDir, const char_T *identifier)
{
  boolean_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(sp, emlrtAlias(isRevDir), &thisId);
  emlrtDestroyArray(&isRevDir);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : boolean_T
 */
static boolean_T d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[7]
 */
static real_T (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
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
 *                const mxArray *Jp_state
 *                const char_T *identifier
 * Return Type  : real_T (*)[7]
 */
  static real_T (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *Jp_state,
  const char_T *identifier))[7]
{
  real_T (*y)[7];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(Jp_state), &thisId);
  emlrtDestroyArray(&Jp_state);
  return y;
}

/*
 * Arguments    : const real_T u[287]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[287])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[2] = { 0, 0 };

  static const int32_T iv1[2] = { 7, 41 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[2])&iv1[0], 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : boolean_T
 */
static boolean_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  boolean_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, (const emlrtMsgIdentifier *)msgId, src, "logical",
    false, 0U, (int32_T *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[3]
 *                int32_T nlhs
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void KnobOperationWAM_api(const mxArray * const prhs[3], int32_T nlhs, const
  mxArray *plhs[2])
{
  real_T (*Jp_cmd)[287];
  const mxArray *prhs_copy_idx_2;
  real_T (*Jp_state)[7];
  boolean_T isRevDir;
  real_T (*PoseKnobWrtWAM)[7];
  real_T success;
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  Jp_cmd = (real_T (*)[287])mxMalloc(sizeof(real_T [287]));
  prhs_copy_idx_2 = emlrtProtectR2012b(prhs[2], 2, false, -1);

  /* Marshall function inputs */
  Jp_state = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "Jp_state");
  isRevDir = c_emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "isRevDir");
  PoseKnobWrtWAM = emlrt_marshallIn(&st, emlrtAlias(prhs_copy_idx_2),
    "PoseKnobWrtWAM");

  /* Invoke the target function */
  KnobOperationWAM(*Jp_state, isRevDir, *PoseKnobWrtWAM, *Jp_cmd, &success);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*Jp_cmd);
  if (nlhs > 1) {
    plhs[1] = b_emlrt_marshallOut(success);
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void KnobOperationWAM_atexit(void)
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
  KnobOperationWAM_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void KnobOperationWAM_initialize(void)
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
void KnobOperationWAM_terminate(void)
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
 * File trailer for _coder_KnobOperationWAM_api.c
 *
 * [EOF]
 */
