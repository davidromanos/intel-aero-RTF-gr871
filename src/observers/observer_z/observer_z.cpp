//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_z.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 11:48:39
//

// Include Files
#include "rt_nonfinite.h"
#include "observer_z.h"

// Variable Definitions
static double states[3];

// Function Definitions

//
// Initialize State Transition Matrix
// Arguments    : double meas
//                double u
//                double y[4]
// Return Type  : void
//
void observer_z(double meas, double u, double y[4])
{
  double a[3];
  double b_a[3];
  int i0;
  double d0;
  int i1;
  static const signed char c_a[3] = { 0, 0, 1 };

  static const double d_a[3] = { 0.8639, 0.9969, 0.0 };

  static const double e_a[9] = { 1.0, 0.0, 0.0, 0.0473, 1.0, 0.0, 0.0, 0.7913,
    0.0 };

  double b_u[2];
  double f_a[4];
  double g_a[4];
  static const double h_a[9] = { 0.8639, 0.9969, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  static const double i_a[12] = { 0.1833, 0.1833, -0.9969, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 1.0 };

  static const double j_a[8] = { 0.0, 0.0, 0.0, 0.0, 0.8167, 0.8167, 0.9969,
    -0.0 };

  // Initialize measurement matrix
  // Initialize matrix for state feedback
  //  derived using current oserver
  //  Store the states from time to time.
  for (i0 = 0; i0 < 3; i0++) {
    d0 = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      d0 += e_a[i0 + 3 * i1] * states[i1];
    }

    a[i0] = (d0 + (double)c_a[i0] * u) + d_a[i0] * meas;
    b_a[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      b_a[i0] += h_a[i0 + 3 * i1] * states[i1];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    states[i0] = a[i0] - b_a[i0];
  }

  //  Advance the system one timestep
  b_u[0] = u;
  b_u[1] = meas;
  for (i0 = 0; i0 < 4; i0++) {
    f_a[i0] = 0.0;
    for (i1 = 0; i1 < 3; i1++) {
      f_a[i0] += i_a[i0 + (i1 << 2)] * states[i1];
    }

    g_a[i0] = 0.0;
    for (i1 = 0; i1 < 2; i1++) {
      g_a[i0] += j_a[i0 + (i1 << 2)] * b_u[i1];
    }

    y[i0] = f_a[i0] + g_a[i0];
  }

  //  Output equation for current observer
}

//
// Arguments    : void
// Return Type  : void
//
void observer_z_init()
{
  int i;

  //  Initialize the state vector the first time the function is used
  for (i = 0; i < 3; i++) {
    states[i] = 0.0;
  }
}

//
// File trailer for observer_z.cpp
//
// [EOF]
//
