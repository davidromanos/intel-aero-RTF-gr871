/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekf_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 16-May-2017 14:57:32
 */

#ifndef _CODER_EKF_API_H
#define _CODER_EKF_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_ekf_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void ekf(uint8_T fastslam_on, real_T fastslam[4], real_T C_fs[16], real_T
                PX4[3], real_T roll_ref, real_T pitch_ref, real_T yaw_ref,
                real_T thrust_ref, real_T est[19], real_T Pout[9], real_T
                *VarYaw);
extern void ekf_api(const mxArray *prhs[8], const mxArray *plhs[3]);
extern void ekf_atexit(void);
extern void ekf_initialize(void);
extern void ekf_terminate(void);
extern void ekf_xil_terminate(void);

#endif

/*
 * File trailer for _coder_ekf_api.h
 *
 * [EOF]
 */
