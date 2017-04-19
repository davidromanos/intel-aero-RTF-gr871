//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_ydot.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 09:17:46
//

// Include Files
#include "rt_nonfinite.h"
#include "observer_ydot.h"

// Variable Definitions
static double states[5];

// Function Definitions

//
// Initialize State Transition Matrix
// Arguments    : const double meas[2]
//                double u
//                double y[5]
// Return Type  : void
//
void observer_ydot(const double meas[2], double u, double y[5])
{
  double a[5];
  double b_a[5];
  int i0;
  double d0;
  int i1;
  static const double c_a[25] = { 1.0, 0.0, 0.0, 0.0, 0.0, -0.4027, 0.261, 1.0,
    0.0, 0.0, 0.197, 0.2374, 0.0, 1.0, 0.0, 0.0828, 0.1581, 0.0, 0.0, 1.0, 0.0,
    0.0646, 0.0, 0.0, 0.0 };

  double d1;
  static const double d_a[10] = { 0.113, -0.004, -0.0042, -0.008, -0.0047,
    -0.0388, 0.0116, 0.0838, -0.0462, -0.0091 };

  static const signed char e_a[25] = { 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1 };

  static const signed char f_a[5] = { 0, 1, 0, 0, 0 };

  static const double g_a[25] = { 0.113, -0.004, -0.0042, -0.008, -0.0047,
    -0.033399040000000005, 0.00998528, 0.07213504, -0.03976896, -0.00783328,
    0.016338679999999998, -0.0048847599999999993, -0.035288179999999995,
    0.019454819999999998, 0.00383201, 0.0068676, -0.0020532, -0.0148326,
    0.0081774, 0.0016107, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Initialize measurement matrix
  // Initialize matrix for state feedback
  //  Optimal observer gain
  //  Store the states from time to time.
  for (i0 = 0; i0 < 5; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 5; i1++) {
      d0 += c_a[i0 + 5 * i1] * states[i1];
    }

    a[i0] = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      a[i0] += d_a[i0 + 5 * i1] * meas[i1];
    }

    d1 = 0.0;
    for (i1 = 0; i1 < 5; i1++) {
      d1 += g_a[i0 + 5 * i1] * states[i1];
    }

    b_a[i0] = ((d0 + (double)f_a[i0] * u) + a[i0]) - d1;
  }

  for (i0 = 0; i0 < 5; i0++) {
    states[i0] = b_a[i0];
  }

  //  Advance the system one timestep
  for (i0 = 0; i0 < 5; i0++) {
    y[i0] = 0.0;
    for (i1 = 0; i1 < 5; i1++) {
      y[i0] += (double)e_a[i0 + 5 * i1] * states[i1];
    }
  }

  //  Output equation
}

//
// Arguments    : void
// Return Type  : void
//
void observer_ydot_init()
{
  int i;

  //  Initialize the state vector the first time the function is used
  for (i = 0; i < 5; i++) {
    states[i] = 0.0;
  }
}

//
// File trailer for observer_ydot.cpp
//
// [EOF]
//
