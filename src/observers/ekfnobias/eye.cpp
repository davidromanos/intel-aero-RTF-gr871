//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: eye.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 15-May-2017 16:40:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ekfnobias.h"
#include "eye.h"

// Function Definitions

//
// Arguments    : double I[289]
// Return Type  : void
//
void eye(double I[289])
{
  int k;
  memset(&I[0], 0, 289U * sizeof(double));
  for (k = 0; k < 17; k++) {
    I[k + 17 * k] = 1.0;
  }
}

//
// File trailer for eye.cpp
//
// [EOF]
//
