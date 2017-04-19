//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_ydot_initialize.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 09:17:46
//

// Include Files
#include "rt_nonfinite.h"
#include "observer_ydot.h"
#include "observer_ydot_initialize.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void observer_ydot_initialize()
{
  rt_InitInfAndNaN(8U);
  observer_ydot_init();
}

//
// File trailer for observer_ydot_initialize.cpp
//
// [EOF]
//
