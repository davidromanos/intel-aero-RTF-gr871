//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: observer_ydot.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 12-Apr-2017 09:17:46
//
#ifndef OBSERVER_YDOT_H
#define OBSERVER_YDOT_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "observer_ydot_types.h"

// Function Declarations
extern void observer_ydot(const double meas[2], double u, double y[5]);
extern void observer_ydot_init();

#endif

//
// File trailer for observer_ydot.h
//
// [EOF]
//
