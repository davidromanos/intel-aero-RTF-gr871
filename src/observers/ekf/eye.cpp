//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 26-Apr-2017 14:21:39
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[4]
// Return Type  : void
//
void b_eye(double I[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    I[k] = 0.0;
  }

  for (k = 0; k < 2; k++) {
    I[k + (k << 1)] = 1.0;
  }
}

//
// Arguments    : double I[256]
// Return Type  : void
//
void c_eye(double I[256])
{
  int k;
  memset(&I[0], 0, sizeof(double) << 8);
  for (k = 0; k < 16; k++) {
    I[k + (k << 4)] = 1.0;
  }
}

//
// Arguments    : double I[256]
// Return Type  : void
//
void eye(double I[256])
{
  int k;
  memset(&I[0], 0, sizeof(double) << 8);
  for (k = 0; k < 16; k++) {
    I[k + (k << 4)] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
