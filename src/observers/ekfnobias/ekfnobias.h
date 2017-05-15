//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekfnobias.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 15-May-2017 17:33:47
//
#ifndef EKFNOBIAS_H
#define EKFNOBIAS_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ekfnobias_types.h"

// Function Declarations
extern void ekfnobias(unsigned char fastslam_on, const double fastslam[4],
                      double C_fs[16], const double PX4[3], double roll_ref,
                      double pitch_ref, double yaw_ref, double thrust_ref,
                      double est[17], double Pout[9], double *VarYaw);
extern void ekfnobias_init();

#endif

//
// File trailer for ekfnobias.h
//
// [EOF]
//
