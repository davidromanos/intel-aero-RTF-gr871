//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: uunwrap.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 15-May-2017 17:33:47
//

// Include Files
#include "rt_nonfinite.h"
#include "ekfnobias.h"
#include "uunwrap.h"

// Function Definitions

//
// Arguments    : double in
//                double state
//                double oldinput
//                double *out
//                double *oldin
// Return Type  : void
//
void uunwrap(double in, double state, double oldinput, double *out, double
             *oldin)
{
  if (std::abs(in - oldinput) > 4.71238898038469) {
    if (in - oldinput > 0.0) {
      state = (state + (in - oldinput)) - 6.2831853071795862;
    } else {
      state = (state + (in - oldinput)) + 6.2831853071795862;
    }
  } else {
    state = (state + in) - oldinput;
  }

  *oldin = in;
  *out = state;
}

//
// File trailer for uunwrap.cpp
//
// [EOF]
//
