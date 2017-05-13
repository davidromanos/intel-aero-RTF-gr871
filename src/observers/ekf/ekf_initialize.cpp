//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf_initialize.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 13-May-2017 13:22:22
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "ekf_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void ekf_initialize()
{
  rt_InitInfAndNaN(8U);
  ekf_init();
}

//
// File trailer for ekf_initialize.cpp
//
// [EOF]
//
