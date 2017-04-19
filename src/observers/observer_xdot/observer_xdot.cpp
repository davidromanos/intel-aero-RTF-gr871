//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_xdot.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 09:16:19
//

// Include Files
#include "rt_nonfinite.h"
#include "observer_xdot.h"

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
void observer_xdot(const double meas[2], double u, double y[5])
{
  double a[5];
  double b_a[5];
  int i0;
  double d0;
  int i1;
  static const double c_a[25] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0907, 1.1869, 1.0,
    0.0, 0.0, 0.091, -0.3037, 0.0, 1.0, 0.0, -0.1452, 0.1289, 0.0, 0.0, 1.0, 0.0,
    -0.1045, 0.0, 0.0, 0.0 };

  double d1;
  static const double d_a[10] = { 0.3128, 0.0725, 0.0774, 0.0855, 0.0883, 0.1142,
    0.3685, 0.4226, 0.3522, -0.1852 };

  static const signed char e_a[25] = { 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1 };

  static const signed char f_a[5] = { 0, 1, 0, 0, 0 };

  static const double g_a[25] = { 0.3128, 0.0725, 0.0774, 0.0855, 0.0883,
    0.02213196, 0.0714153, 0.08189988, 0.06825636, -0.03589176,
    0.022200479999999998, 0.071636399999999989, 0.08215344, 0.06846768,
    -0.03600288, -0.035436260000000004, -0.11434555, -0.13113278,
    -0.10928766000000001, 0.057467560000000008, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // Initialize measurement matrix
  // Initialize matrix for state feedback
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
void observer_xdot_init()
{
  int i;

  //  Initialize the state vector the first time the function is used
  for (i = 0; i < 5; i++) {
    states[i] = 0.0;
  }
}

//
// File trailer for observer_xdot.cpp
//
// [EOF]
//
