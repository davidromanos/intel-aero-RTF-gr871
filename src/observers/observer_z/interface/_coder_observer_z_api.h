/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_observer_z_api.h
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 12-Apr-2017 11:48:39
 */

#ifndef _CODER_OBSERVER_Z_API_H
#define _CODER_OBSERVER_Z_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_observer_z_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void observer_z(real_T meas, real_T u, real_T y[4]);
extern void observer_z_api(const mxArray * const prhs[2], const mxArray *plhs[1]);
extern void observer_z_atexit(void);
extern void observer_z_initialize(void);
extern void observer_z_terminate(void);
extern void observer_z_xil_terminate(void);

#endif

/*
 * File trailer for _coder_observer_z_api.h
 *
 * [EOF]
 */
