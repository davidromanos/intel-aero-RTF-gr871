//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mod.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 13-May-2017 13:22:22
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "mod.h"

// Function Definitions

//
// Arguments    : double x
// Return Type  : double
//
double b_mod(double x)
{
  double r;
  boolean_T rEQ0;
  double q;
  if ((!rtIsInf(x)) && (!rtIsNaN(x))) {
    if (x == 0.0) {
      r = 0.0;
    } else {
      r = std::fmod(x, 6.2831853071795862);
      rEQ0 = (r == 0.0);
      if (!rEQ0) {
        q = std::abs(x / 6.2831853071795862);
        rEQ0 = (std::abs(q - std::floor(q + 0.5)) <= 2.2204460492503131E-16 * q);
      }

      if (rEQ0) {
        r = 0.0;
      } else {
        if (x < 0.0) {
          r += 6.2831853071795862;
        }
      }
    }
  } else {
    r = rtNaN;
  }

  return r;
}

//
// File trailer for mod.cpp
//
// [EOF]
//
