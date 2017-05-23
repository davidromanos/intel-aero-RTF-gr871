//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 23-May-2017 09:55:53
//
#ifndef EKF_H
#define EKF_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "ekf_types.h"

// Function Declarations
extern void ekf(unsigned char fastslam_on, const double fastslam[4], const
                double C_fs[16], const double PX4[3], double roll_ref, double
                pitch_ref, double yaw_ref, double thrust_ref, double est[19],
                double Pout[9], double *VarYaw);
extern void ekf_init();

#endif

//
// File trailer for ekf.h
//
// [EOF]
//
