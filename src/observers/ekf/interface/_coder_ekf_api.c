/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekf_api.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 23-May-2017 09:55:53
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_ekf_api.h"
#include "_coder_ekf_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131450U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "ekf",                               /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static uint8_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static const mxArray *b_emlrt_marshallOut(const real_T u[9]);
static real_T (*c_emlrt_marshallIn(const mxArray *fastslam, const char_T
  *identifier))[4];
static const mxArray *c_emlrt_marshallOut(const real_T u);
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[4];
static real_T (*e_emlrt_marshallIn(const mxArray *C_fs, const char_T *identifier))
  [16];
static uint8_T emlrt_marshallIn(const mxArray *fastslam_on, const char_T
  *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[19]);
static real_T (*f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[16];
static real_T (*g_emlrt_marshallIn(const mxArray *PX4, const char_T *identifier))
  [3];
static real_T (*h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[3];
static real_T i_emlrt_marshallIn(const mxArray *roll_ref, const char_T
  *identifier);
static real_T j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId);
static uint8_T k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);
static real_T (*l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[4];
static real_T (*m_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[16];
static real_T (*n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3];
static real_T o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId);

/* Function Definitions */

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : uint8_T
 */
static uint8_T b_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  uint8_T y;
  y = k_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const real_T u[9]
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const real_T u[9])
{
  const mxArray *y;
  const mxArray *m1;
  static const int32_T iv2[2] = { 0, 0 };

  static const int32_T iv3[2] = { 3, 3 };

  y = NULL;
  m1 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m1, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m1, *(int32_T (*)[2])&iv3[0], 2);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const mxArray *fastslam
 *                const char_T *identifier
 * Return Type  : real_T (*)[4]
 */
static real_T (*c_emlrt_marshallIn(const mxArray *fastslam, const char_T
  *identifier))[4]
{
  real_T (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(fastslam), &thisId);
  emlrtDestroyArray(&fastslam);
  return y;
}
/*
 * Arguments    : const real_T u
 * Return Type  : const mxArray *
 */
  static const mxArray *c_emlrt_marshallOut(const real_T u)
{
  const mxArray *y;
  const mxArray *m2;
  y = NULL;
  m2 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[4]
 */
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[4]
{
  real_T (*y)[4];
  y = l_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const mxArray *C_fs
 *                const char_T *identifier
 * Return Type  : real_T (*)[16]
 */
  static real_T (*e_emlrt_marshallIn(const mxArray *C_fs, const char_T
  *identifier))[16]
{
  real_T (*y)[16];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(emlrtAlias(C_fs), &thisId);
  emlrtDestroyArray(&C_fs);
  return y;
}

/*
 * Arguments    : const mxArray *fastslam_on
 *                const char_T *identifier
 * Return Type  : uint8_T
 */
static uint8_T emlrt_marshallIn(const mxArray *fastslam_on, const char_T
  *identifier)
{
  uint8_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(emlrtAlias(fastslam_on), &thisId);
  emlrtDestroyArray(&fastslam_on);
  return y;
}

/*
 * Arguments    : const real_T u[19]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[19])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 0 };

  static const int32_T iv1[1] = { 19 };

  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m0, *(int32_T (*)[1])&iv1[0], 1);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[16]
 */
static real_T (*f_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[16]
{
  real_T (*y)[16];
  y = m_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const mxArray *PX4
 *                const char_T *identifier
 * Return Type  : real_T (*)[3]
 */
  static real_T (*g_emlrt_marshallIn(const mxArray *PX4, const char_T
  *identifier))[3]
{
  real_T (*y)[3];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = h_emlrt_marshallIn(emlrtAlias(PX4), &thisId);
  emlrtDestroyArray(&PX4);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T (*)[3]
 */
static real_T (*h_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[3]
{
  real_T (*y)[3];
  y = n_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}
/*
 * Arguments    : const mxArray *roll_ref
 *                const char_T *identifier
 * Return Type  : real_T
 */
  static real_T i_emlrt_marshallIn(const mxArray *roll_ref, const char_T
  *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = (const char *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = j_emlrt_marshallIn(emlrtAlias(roll_ref), &thisId);
  emlrtDestroyArray(&roll_ref);
  return y;
}

/*
 * Arguments    : const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T j_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId)
{
  real_T y;
  y = o_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : uint8_T
 */
static uint8_T k_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier
  *msgId)
{
  uint8_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "uint8", false, 0U,
    &dims);
  ret = *(uint8_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[4]
 */
static real_T (*l_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[4]
{
  real_T (*ret)[4];
  static const int32_T dims[1] = { 4 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  ret = (real_T (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[16]
 */
  static real_T (*m_emlrt_marshallIn(const mxArray *src, const
  emlrtMsgIdentifier *msgId))[16]
{
  real_T (*ret)[16];
  static const int32_T dims[2] = { 4, 4 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims);
  ret = (real_T (*)[16])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T (*)[3]
 */
static real_T (*n_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3]
{
  real_T (*ret)[3];
  static const int32_T dims[1] = { 3 };

  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 1U,
    dims);
  ret = (real_T (*)[3])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
  static real_T o_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 0U,
    &dims);
  ret = *(real_T *)mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[8]
 *                const mxArray *plhs[3]
 * Return Type  : void
 */
void ekf_api(const mxArray * const prhs[8], const mxArray *plhs[3])
{
  real_T (*est)[19];
  real_T (*Pout)[9];
  uint8_T fastslam_on;
  real_T (*fastslam)[4];
  real_T (*C_fs)[16];
  real_T (*PX4)[3];
  real_T roll_ref;
  real_T pitch_ref;
  real_T yaw_ref;
  real_T thrust_ref;
  real_T VarYaw;
  est = (real_T (*)[19])mxMalloc(sizeof(real_T [19]));
  Pout = (real_T (*)[9])mxMalloc(sizeof(real_T [9]));

  /* Marshall function inputs */
  fastslam_on = emlrt_marshallIn(emlrtAliasP((const mxArray *)prhs[0]),
    "fastslam_on");
  fastslam = c_emlrt_marshallIn(emlrtAlias((const mxArray *)prhs[1]), "fastslam");
  C_fs = e_emlrt_marshallIn(emlrtAlias((const mxArray *)prhs[2]), "C_fs");
  PX4 = g_emlrt_marshallIn(emlrtAlias((const mxArray *)prhs[3]), "PX4");
  roll_ref = i_emlrt_marshallIn(emlrtAliasP((const mxArray *)prhs[4]),
    "roll_ref");
  pitch_ref = i_emlrt_marshallIn(emlrtAliasP((const mxArray *)prhs[5]),
    "pitch_ref");
  yaw_ref = i_emlrt_marshallIn(emlrtAliasP((const mxArray *)prhs[6]), "yaw_ref");
  thrust_ref = i_emlrt_marshallIn(emlrtAliasP((const mxArray *)prhs[7]),
    "thrust_ref");

  /* Invoke the target function */
  ekf(fastslam_on, *fastslam, *C_fs, *PX4, roll_ref, pitch_ref, yaw_ref,
      thrust_ref, *est, *Pout, &VarYaw);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*est);
  plhs[1] = b_emlrt_marshallOut(*Pout);
  plhs[2] = c_emlrt_marshallOut(VarYaw);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  ekf_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ekf_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_ekf_api.c
 *
 * [EOF]
 */
