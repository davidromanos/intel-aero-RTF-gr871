//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekfnobias_initialize.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 15-May-2017 17:33:47
//

// Include Files
#include "rt_nonfinite.h"
#include "ekfnobias.h"
#include "ekfnobias_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void ekfnobias_initialize()
{
  rt_InitInfAndNaN(8U);
  ekfnobias_init();
}

//
// File trailer for ekfnobias_initialize.cpp
//
// [EOF]
//
