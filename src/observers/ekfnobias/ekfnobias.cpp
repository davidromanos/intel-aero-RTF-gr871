//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekfnobias.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 15-May-2017 16:40:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ekfnobias.h"
#include "mod.h"
#include "eye.h"
#include "inv.h"
#include "measurementModel.h"
#include "uunwrap.h"

// Variable Definitions
static double refState;
static double refOldinput;
static double states[17];
static double P[289];

// Function Definitions

//
// function est = ekf(fastslam_on, fastslam, C_fs, PX4, roll_ref, pitch_ref, yaw_ref, thrust_ref)
// Arguments    : unsigned char fastslam_on
//                const double fastslam[4]
//                double C_fs[16]
//                const double PX4[3]
//                double roll_ref
//                double pitch_ref
//                double yaw_ref
//                double thrust_ref
//                double est[17]
//                double Pout[9]
//                double *VarYaw
// Return Type  : void
//
void ekfnobias(unsigned char fastslam_on, const double fastslam[4], double C_fs
               [16], const double PX4[3], double roll_ref, double pitch_ref,
               double yaw_ref, double thrust_ref, double est[17], double Pout[9],
               double *VarYaw)
{
  double b_refState;
  double b_refOldinput;
  double H_fs[68];
  int i0;
  double cov[289];
  static const signed char iv0[289] = { 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2 };

  signed char C_PX4[9];
  static const signed char iv1[9] = { 5, 0, 0, 0, 5, 0, 0, 0, 5 };

  int meas_size_idx_0;
  double meas_data[7];
  int H_size_idx_0;
  double R_data[49];
  double H_data[119];
  static const double H_PX4[51] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.1938, 0.0, 0.0, -0.1944, 0.0, 0.0, 0.3103, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.8608, 0.0, 0.0, 0.4211, 0.0, 0.0, 0.177, 0.0, 0.0, -0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  double b_fastslam[7];
  double b_pitch_ref[4];
  int c;
  double b_C_fs[49];
  double a[17];
  double b_a[17];
  static const double c_a[289] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0477, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0907,
    1.1869, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.091, -0.3037, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.1452, 0.1289, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1045, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0477, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, -0.4027, 0.261, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.197, 0.2374, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0828, 0.1581, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0646, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.944, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0473, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7913, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    1.0 };

  int cr;
  double states_p[17];
  static const double d_a[68] = { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05639, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 };

  int br;
  int ic;
  double b_H_fs[119];
  double e_a[289];
  int k;
  int ar;
  int ib;
  double P_p[289];
  static const double b[289] = { 1.0, 0.0, 0.0477, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0477, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0907,
    0.091, -0.1452, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.1869, -0.3037, 0.1289, -0.1045, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, -0.4027, 0.197, 0.0828, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.261, 0.2374, 0.1581, 0.0646, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.944, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0473, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.7913, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  int ia;
  double L_data[119];
  int C_size[2];
  double C_data[49];
  int R_size[2];
  double dv0[289];
  static const signed char iv2[3] = { 2, 7, 14 };

  // Initialize constant matrices
  uunwrap(yaw_ref, refState, refOldinput, &b_refState, &b_refOldinput);
  refState = b_refState;
  refOldinput = b_refOldinput;
  if (rtIsNaN(refState)) {
    refState = 0.0;
  }

  if (rtIsNaN(refOldinput)) {
    refOldinput = 0.0;
  }

  measurementModel(H_fs);

  //  Number of states
  // weight matrix for states noise
  for (i0 = 0; i0 < 289; i0++) {
    cov[i0] = iv0[i0];
  }

  cov[36] = 150.0;
  cov[126] = 150.0;
  cov[216] = 1.0;
  cov[252] = 10.0;

  //  z dot
  cov[270] = 50.0;
  cov[288] = 0.01;

  //  z bias
  // constant covariance noise matrix PX4
  for (i0 = 0; i0 < 9; i0++) {
    C_PX4[i0] = iv1[i0];
  }

  C_PX4[8] = 1;
  C_fs[15] = 1.0;

  // C_fs(1,1) = 1;
  // input vector
  //  Store the states from time to time.
  //  Store the covariance from time to time.
  // measurements and measurements covariance
  if (fastslam_on == 1) {
    H_fs[0] = std::cos(states[12]);
    H_fs[4] = -std::sin(states[12]);
    H_fs[1] = std::sin(states[12]);
    H_fs[5] = std::cos(states[12]);
    for (i0 = 0; i0 < 4; i0++) {
      b_fastslam[i0] = fastslam[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_fastslam[i0 + 4] = PX4[i0];
    }

    meas_size_idx_0 = 7;
    for (i0 = 0; i0 < 7; i0++) {
      meas_data[i0] = b_fastslam[i0];
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (c = 0; c < 4; c++) {
        b_C_fs[c + 7 * i0] = C_fs[c + (i0 << 2)];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (c = 0; c < 4; c++) {
        b_C_fs[c + 7 * (i0 + 4)] = 0.0;
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (c = 0; c < 3; c++) {
        b_C_fs[(c + 7 * i0) + 4] = 0.0;
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (c = 0; c < 3; c++) {
        b_C_fs[(c + 7 * (i0 + 4)) + 4] = C_PX4[c + 3 * i0];
      }
    }

    for (i0 = 0; i0 < 7; i0++) {
      for (c = 0; c < 7; c++) {
        R_data[c + 7 * i0] = b_C_fs[c + 7 * i0];
      }
    }

    H_size_idx_0 = 7;
    for (i0 = 0; i0 < 17; i0++) {
      for (c = 0; c < 4; c++) {
        b_H_fs[c + 7 * i0] = H_fs[c + (i0 << 2)];
      }

      for (c = 0; c < 3; c++) {
        b_H_fs[(c + 7 * i0) + 4] = H_PX4[c + 3 * i0];
      }

      for (c = 0; c < 7; c++) {
        H_data[c + 7 * i0] = b_H_fs[c + 7 * i0];
      }
    }
  } else {
    meas_size_idx_0 = 3;
    for (i0 = 0; i0 < 3; i0++) {
      meas_data[i0] = PX4[i0];
    }

    for (i0 = 0; i0 < 9; i0++) {
      R_data[i0] = C_PX4[i0];
    }

    H_size_idx_0 = 3;
    memcpy(&H_data[0], &H_PX4[0], 51U * sizeof(double));
  }

  // Prediction step
  // predicted states with linear model
  b_pitch_ref[0] = pitch_ref;
  b_pitch_ref[1] = roll_ref;
  b_pitch_ref[2] = yaw_ref;
  b_pitch_ref[3] = thrust_ref - 0.587;

  // states_p(13) = mod(states_p(13)+pi,2*pi)-pi;
  //
  // predicted covariance estimate with linearised model
  for (i0 = 0; i0 < 17; i0++) {
    a[i0] = 0.0;
    for (c = 0; c < 17; c++) {
      a[i0] += c_a[i0 + 17 * c] * states[c];
    }

    b_a[i0] = 0.0;
    for (c = 0; c < 4; c++) {
      b_a[i0] += d_a[i0 + 17 * c] * b_pitch_ref[c];
    }

    states_p[i0] = a[i0] + b_a[i0];
    for (c = 0; c < 17; c++) {
      e_a[i0 + 17 * c] = 0.0;
      for (k = 0; k < 17; k++) {
        e_a[i0 + 17 * c] += c_a[i0 + 17 * k] * P[k + 17 * c];
      }
    }

    for (c = 0; c < 17; c++) {
      b_refState = 0.0;
      for (k = 0; k < 17; k++) {
        b_refState += e_a[i0 + 17 * k] * b[k + 17 * c];
      }

      P_p[i0 + 17 * c] = b_refState + cov[i0 + 17 * c];
    }
  }

  // Update step
  // measurement residual with linear model (H is always linear here)
  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    b_fastslam[i0] = 0.0;
  }

  cr = 0;
  while (cr <= 0) {
    for (ic = 1; ic <= H_size_idx_0; ic++) {
      b_fastslam[ic - 1] = 0.0;
    }

    cr = H_size_idx_0;
  }

  br = 0;
  cr = 0;
  while (cr <= 0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 17; ib++) {
      if (states_p[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic + 1 <= H_size_idx_0; ic++) {
          ia++;
          b_fastslam[ic] += states_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 17;
    cr = H_size_idx_0;
  }

  for (i0 = 0; i0 < meas_size_idx_0; i0++) {
    meas_data[i0] -= b_fastslam[i0];
  }

  if (fastslam_on == 1) {
    meas_data[3] = b_mod(meas_data[3] + 3.1415926535897931) - 3.1415926535897931;
  }

  meas_data[meas_size_idx_0 - 1] = b_mod(meas_data[meas_size_idx_0 - 1] +
    3.1415926535897931) - 3.1415926535897931;

  // residual covariance with linear model
  for (i0 = 0; i0 < 17; i0++) {
    for (c = 0; c < H_size_idx_0; c++) {
      b_H_fs[c + H_size_idx_0 * i0] = 0.0;
    }
  }

  c = H_size_idx_0 << 4;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    i0 = cr + H_size_idx_0;
    for (ic = cr; ic + 1 <= i0; ic++) {
      b_H_fs[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 17; ib++) {
      if (P_p[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          b_H_fs[ic] += P_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 17;
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 17; c++) {
      L_data[c + 17 * i0] = H_data[i0 + H_size_idx_0 * c];
    }
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < H_size_idx_0; c++) {
      b_C_fs[c + H_size_idx_0 * i0] = 0.0;
    }
  }

  c = H_size_idx_0 * (H_size_idx_0 - 1);
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    i0 = cr + H_size_idx_0;
    for (ic = cr; ic + 1 <= i0; ic++) {
      b_C_fs[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 17; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          b_C_fs[ic] += L_data[ib] * b_H_fs[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 17;
  }

  // optimal Kalman gain
  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 17; c++) {
      L_data[c + 17 * i0] = H_data[i0 + H_size_idx_0 * c];
    }
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    memset(&b_H_fs[i0 * 17], 0, 17U * sizeof(double));
  }

  c = 17 * (H_size_idx_0 - 1);
  for (cr = 0; cr <= c; cr += 17) {
    for (ic = cr; ic + 1 <= cr + 17; ic++) {
      b_H_fs[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 17) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 17; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 17; ic++) {
          ia++;
          b_H_fs[ic] += L_data[ib] * P_p[ia];
        }
      }

      ar += 17;
    }

    br += 17;
  }

  C_size[0] = H_size_idx_0;
  C_size[1] = H_size_idx_0;
  k = H_size_idx_0 * H_size_idx_0;
  for (i0 = 0; i0 < k; i0++) {
    C_data[i0] = b_C_fs[i0] + R_data[i0];
  }

  inv(C_data, C_size, R_data, R_size);
  k = (signed char)R_size[1];
  for (i0 = 0; i0 < k; i0++) {
    memset(&L_data[i0 * 17], 0, 17U * sizeof(double));
  }

  c = 17 * (R_size[1] - 1);
  for (cr = 0; cr <= c; cr += 17) {
    for (ic = cr; ic + 1 <= cr + 17; ic++) {
      L_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 17) {
    ar = -1;
    i0 = br + H_size_idx_0;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (R_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 17; ic++) {
          ia++;
          L_data[ic] += R_data[ib] * b_H_fs[ia];
        }
      }

      ar += 17;
    }

    br += H_size_idx_0;
  }

  // update states
  memset(&states[0], 0, 17U * sizeof(double));
  ar = -1;
  for (ib = 0; ib + 1 <= (signed char)R_size[1]; ib++) {
    if (meas_data[ib] != 0.0) {
      ia = ar;
      for (ic = 0; ic < 17; ic++) {
        ia++;
        states[ic] += meas_data[ib] * L_data[ia];
      }
    }

    ar += 17;
  }

  for (i0 = 0; i0 < 17; i0++) {
    states[i0] += states_p[i0];
  }

  // update covariance
  k = (signed char)R_size[1];
  memset(&cov[0], 0, 289U * sizeof(double));
  for (cr = 0; cr <= 273; cr += 17) {
    for (ic = cr; ic + 1 <= cr + 17; ic++) {
      cov[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 273; cr += 17) {
    ar = -1;
    i0 = br + k;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (H_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 17; ic++) {
          ia++;
          cov[ic] += H_data[ib] * L_data[ia];
        }
      }

      ar += 17;
    }

    br += k;
  }

  eye(e_a);
  for (i0 = 0; i0 < 17; i0++) {
    for (c = 0; c < 17; c++) {
      dv0[c + 17 * i0] = e_a[c + 17 * i0] - cov[c + 17 * i0];
    }
  }

  // Output of the function (for state feedback)
  // states(13) = mod(states(13)+pi,2*pi)-pi;
  // refState = mod(refState+pi,2*pi)-pi;
  for (k = 0; k < 17; k++) {
    for (i0 = 0; i0 < 17; i0++) {
      P[k + 17 * i0] = 0.0;
      for (c = 0; c < 17; c++) {
        P[k + 17 * i0] += dv0[k + 17 * c] * P_p[c + 17 * i0];
      }
    }

    est[k] = states[k];
  }

  est[12] = b_mod(states[12] + 3.1415926535897931) - 3.1415926535897931;

  //  states_pp = states_p;
  //  states_pp(13) = mod(states_pp(13)+pi,2*pi)-pi;
  for (i0 = 0; i0 < 3; i0++) {
    Pout[3 * i0] = P[2 + 17 * iv2[i0]];
    Pout[1 + 3 * i0] = P[7 + 17 * iv2[i0]];
    Pout[2 + 3 * i0] = P[14 + 17 * iv2[i0]];
  }

  *VarYaw = P[216];
}

//
// Arguments    : void
// Return Type  : void
//
void ekfnobias_init()
{
  int k;
  refState = 0.0;
  refOldinput = 0.0;

  //  Initialize the state vector the first time the function is used
  memset(&states[0], 0, 17U * sizeof(double));

  // [meas(1);meas(2);0;0;0;0;0;0;0;0;0;0;0;meas(6);0;0];
  //  Initialize the covariance vector the first time the function is used
  memset(&P[0], 0, 289U * sizeof(double));
  for (k = 0; k < 17; k++) {
    P[k + 17 * k] = 1.0;
  }
}

//
// File trailer for ekfnobias.cpp
//
// [EOF]
//
