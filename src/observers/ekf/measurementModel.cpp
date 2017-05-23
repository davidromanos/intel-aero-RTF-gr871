//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: measurementModel.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 23-May-2017 11:16:32
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "measurementModel.h"

// Function Definitions

//
// Cpitch = [0 0.1938 0.1944 -0.3103];
//  Cpitch = [0 Cpitch];
//  Croll = [0 0.8608 -0.4211 -0.1770];
//  Croll = [0 Croll];
//  Cyaw = 1;
//  H_PX4 = blkdiag(Cpitch,Croll,Cyaw);
//  H_PX4 = [zeros(3,2) H_PX4 zeros(3,6)];
//  H_PX4(1,18) = 1;
//  H_PX4(2,19) = 1;
//
//
//  H_fs0 = zeros(4,19);
//  H_fs0(1:2,1:2) = eye(2);
//  H_fs0(3,14) = 1;
//  H_fs0(4,13) = 1;
// Arguments    : double H_fs0[76]
//                double H_PX4[57]
// Return Type  : void
//
void measurementModel(double H_fs0[76], double H_PX4[57])
{
  double b_H_PX4[33];
  int k;
  static const double varargin_1[5] = { 0.0, 0.1938, 0.1944, -0.3103, 0.0 };

  static const double varargin_2[5] = { 0.0, 0.8608, -0.4211, -0.177, 0.0 };

  int i1;
  signed char I[4];
  memset(&b_H_PX4[0], 0, 33U * sizeof(double));
  for (k = 0; k < 5; k++) {
    b_H_PX4[3 * k] = varargin_1[k];
    b_H_PX4[1 + 3 * (5 + k)] = varargin_2[k];
  }

  b_H_PX4[32] = 1.0;
  for (k = 0; k < 2; k++) {
    for (i1 = 0; i1 < 3; i1++) {
      H_PX4[i1 + 3 * k] = 0.0;
    }
  }

  for (k = 0; k < 11; k++) {
    for (i1 = 0; i1 < 3; i1++) {
      H_PX4[i1 + 3 * (k + 2)] = b_H_PX4[i1 + 3 * k];
    }
  }

  for (k = 0; k < 6; k++) {
    for (i1 = 0; i1 < 3; i1++) {
      H_PX4[i1 + 3 * (k + 13)] = 0.0;
    }
  }

  H_PX4[51] = 1.0;
  H_PX4[55] = 1.0;
  memset(&H_fs0[0], 0, 76U * sizeof(double));
  for (k = 0; k < 4; k++) {
    I[k] = 0;
  }

  for (k = 0; k < 2; k++) {
    I[k + (k << 1)] = 1;
  }

  for (k = 0; k < 2; k++) {
    for (i1 = 0; i1 < 2; i1++) {
      H_fs0[i1 + (k << 2)] = I[i1 + (k << 1)];
    }
  }

  H_fs0[54] = 1.0;
  H_fs0[51] = 1.0;
}

//
// File trailer for measurementModel.cpp
//
// [EOF]
//
