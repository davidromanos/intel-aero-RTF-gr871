//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_xdot_initialize.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 09:16:19
//

// Include Files
#include "rt_nonfinite.h"
#include "observer_xdot.h"
#include "observer_xdot_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void observer_xdot_initialize()
{
  rt_InitInfAndNaN(8U);
  observer_xdot_init();
}

//
// File trailer for observer_xdot_initialize.cpp
//
// [EOF]
//
