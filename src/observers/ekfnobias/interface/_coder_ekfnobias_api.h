/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_ekfnobias_api.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 15-May-2017 17:33:47
 */

#ifndef _CODER_EKFNOBIAS_API_H
#define _CODER_EKFNOBIAS_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_ekfnobias_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void ekfnobias(uint8_T fastslam_on, real_T fastslam[4], real_T C_fs[16],
                      real_T PX4[3], real_T roll_ref, real_T pitch_ref, real_T
                      yaw_ref, real_T thrust_ref, real_T est[17], real_T Pout[9],
                      real_T *VarYaw);
extern void ekfnobias_api(const mxArray *prhs[8], const mxArray *plhs[3]);
extern void ekfnobias_atexit(void);
extern void ekfnobias_initialize(void);
extern void ekfnobias_terminate(void);
extern void ekfnobias_xil_terminate(void);

#endif

/*
 * File trailer for _coder_ekfnobias_api.h
 *
 * [EOF]
 */
