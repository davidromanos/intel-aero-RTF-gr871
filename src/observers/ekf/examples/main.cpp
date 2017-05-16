//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 16-May-2017 14:57:32
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "main.h"
#include "ekf_terminate.h"
#include "ekf_initialize.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);
static void argInit_4x1_real_T(double result[4]);
static void argInit_4x4_real_T(double result[16]);
static double argInit_real_T();
static unsigned char argInit_uint8_T();
static void main_ekf();

// Function Definitions

//
// Arguments    : double result[3]
// Return Type  : void
//
static void argInit_3x1_real_T(double result[3])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[4]
// Return Type  : void
//
static void argInit_4x1_real_T(double result[4])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : double result[16]
// Return Type  : void
//
static void argInit_4x4_real_T(double result[16])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 4; idx0++) {
    for (idx1 = 0; idx1 < 4; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + (idx1 << 2)] = argInit_real_T();
    }
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : unsigned char
//
static unsigned char argInit_uint8_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_ekf()
{
  double dv1[4];
  double dv2[16];
  double dv3[3];
  double est[19];
  double Pout[9];
  double VarYaw;

  // Initialize function 'ekf' input arguments.
  // Initialize function input argument 'fastslam'.
  // Initialize function input argument 'C_fs'.
  // Initialize function input argument 'PX4'.
  // Call the entry-point 'ekf'.
  argInit_4x1_real_T(dv1);
  argInit_4x4_real_T(dv2);
  argInit_3x1_real_T(dv3);
  ekf(argInit_uint8_T(), dv1, dv2, dv3, argInit_real_T(), argInit_real_T(),
      argInit_real_T(), argInit_real_T(), est, Pout, &VarYaw);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  ekf_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_ekf();

  // Terminate the application.
  // You do not need to do this more than one time.
  ekf_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
