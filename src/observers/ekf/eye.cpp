//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 16-May-2017 14:57:32
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[361]
// Return Type  : void
//
void eye(double I[361])
{
  int k;
  memset(&I[0], 0, 361U * sizeof(double));
  for (k = 0; k < 19; k++) {
    I[k + 19 * k] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
