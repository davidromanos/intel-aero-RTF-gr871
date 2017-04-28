//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 26-Apr-2017 14:21:39
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "eye.h"
#include "inv.h"

// Variable Definitions
static double states[16];
static double P[256];

// Function Definitions

//
// Initialize constant matrices
// linear part of f function
//  % x dot ARX
// Arguments    : unsigned char fastslam_on
//                const double fastslam[6]
//                const double C_fs[36]
//                const double imu[2]
//                double roll_ref
//                double pitch_ref
//                double yaw_ref
//                double thrust_ref
//                double est[16]
// Return Type  : void
//
void ekf(unsigned char fastslam_on, const double fastslam[6], const double C_fs
         [36], const double imu[2], double roll_ref, double pitch_ref, double
         yaw_ref, double thrust_ref, double est[16])
{
  double C_imu[4];
  int meas_size_idx_0;
  int i0;
  double meas_data[8];
  int H_size_idx_0;
  double R_data[64];
  double H_data[128];
  static const double H_imu[32] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1938,
    0.0, 0.1944, 0.0, -0.3103, 0.0, 0.0, 0.0, 0.0, 0.8608, 0.0, -0.4211, 0.0,
    -0.177, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  int c;
  double b_C_fs[64];
  static const double dv0[128] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.1938, 0.0, 0.0, 0.1938, 0.0, 0.0, 0.0, 0.0, 0.1944,
    0.0, 0.0, 0.1944, 0.0, 0.0, 0.0, 0.0, -0.3103, 0.0, 0.0, -0.3103, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.8608, 0.0, 0.0, 0.8608, 0.0, 0.0, 0.0, 0.0, -0.4211, 0.0, 0.0,
    -0.4211, 0.0, 0.0, 0.0, 0.0, -0.177, 0.0, 0.0, -0.177, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  double states_p[16];
  double a[14];
  double F[256];
  double b_a[14];
  static const double c_a[196] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0907, 1.1869, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.091, -0.3037, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.1452, 0.1289, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1045, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4027, 0.261, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.197, 0.2374, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0828, 0.1581, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0646, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9642, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0473, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7913, 0.0 };

  static const double d_a[56] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.04379, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0 };

  static const double dv1[224] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0907, 1.1869, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.091, -0.3037, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.1452, 0.1289, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -0.1045, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4027, 0.261, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.197, 0.2374, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0828, 0.1581, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0646, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9642, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0473, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7913, 0.0 };

  double dv2[256];
  double b_F[256];
  int k;
  int cr;
  double C_data[8];
  double d0;
  int br;
  int ic;
  double P_p[256];
  int ar;
  int ib;
  int ia;
  double y_data[128];
  double L_data[128];
  int C_size[2];
  double b_C_data[64];
  int R_size[2];

  //  %y dot ARX
  //  %yaw ARX model
  //  % z model
  // B matrix for prediction step
  // %x dot
  //  %y dot
  //  %yaw model
  //  %z
  // linear h functions depending on measurements
  //  %x
  //  %y
  //  %z
  //  %roll
  //  %pitch
  // yaw
  //  %roll];
  // pitch
  // weight matrix for states noise
  // constant covariance noise matrix of emu
  b_eye(C_imu);

  // input vector
  // measurements and measurements covariance
  if (fastslam_on == 1) {
    for (i0 = 0; i0 < 6; i0++) {
      meas_data[i0] = fastslam[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      meas_data[i0 + 6] = imu[i0];
    }

    meas_size_idx_0 = 8;
    for (i0 = 0; i0 < 6; i0++) {
      for (c = 0; c < 6; c++) {
        b_C_fs[c + (i0 << 3)] = C_fs[c + 6 * i0];
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (c = 0; c < 6; c++) {
        b_C_fs[c + ((i0 + 6) << 3)] = 0.0;
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      for (c = 0; c < 2; c++) {
        b_C_fs[(c + (i0 << 3)) + 6] = 0.0;
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (c = 0; c < 2; c++) {
        b_C_fs[(c + ((i0 + 6) << 3)) + 6] = C_imu[c + (i0 << 1)];
      }
    }

    for (i0 = 0; i0 < 8; i0++) {
      memcpy(&R_data[i0 << 3], &b_C_fs[i0 << 3], sizeof(double) << 3);
    }

    H_size_idx_0 = 8;
    memcpy(&H_data[0], &dv0[0], sizeof(double) << 7);
  } else {
    meas_size_idx_0 = 2;
    for (i0 = 0; i0 < 2; i0++) {
      meas_data[i0] = imu[i0];
    }

    for (i0 = 0; i0 < 4; i0++) {
      R_data[i0] = C_imu[i0];
    }

    H_size_idx_0 = 2;
    memcpy(&H_data[0], &H_imu[0], sizeof(double) << 5);
  }

  //  Store the states from time to time.
  //  Store the covariance from time to time.
  // Prediction step
  // predicted states with nonlinear model
  // predicted x
  // predited y
  // predicted states constantly linear
  // predicted states vector
  C_imu[0] = pitch_ref;
  C_imu[1] = roll_ref;
  C_imu[2] = yaw_ref + 1.5707963267948966;
  C_imu[3] = thrust_ref - 0.587;
  states_p[0] = (states[0] + 0.0477 * std::cos(states[12]) * states[2]) + 0.0477
    * std::sin(states[12]) * states[7];
  states_p[1] = (states[1] + 0.0477 * std::cos(states[12]) * states[7]) - 0.0477
    * std::sin(states[12]) * states[2];
  for (i0 = 0; i0 < 14; i0++) {
    a[i0] = 0.0;
    for (c = 0; c < 14; c++) {
      a[i0] += c_a[i0 + 14 * c] * states[2 + c];
    }

    b_a[i0] = 0.0;
    for (c = 0; c < 4; c++) {
      b_a[i0] += d_a[i0 + 14 * c] * C_imu[c];
    }

    states_p[i0 + 2] = a[i0] + b_a[i0];
  }

  // linearised f nonlinear part (jacobian)
  //  %x lin
  // y lin
  F[0] = 1.0;
  F[16] = 0.0;
  F[32] = 0.0477 * std::cos(states[12]);
  F[48] = 0.0;
  F[64] = 0.0;
  F[80] = 0.0;
  F[96] = 0.0;
  F[112] = 0.0477 * std::sin(states[12]);
  F[128] = 0.0;
  F[144] = 0.0;
  F[160] = 0.0;
  F[176] = 0.0;
  F[192] = -0.0477 * states[2] * std::sin(states[12]) + 0.0477 * states[7] * std::
    cos(states[12]);
  F[208] = 0.0;
  F[224] = 0.0;
  F[240] = 0.0;
  F[1] = 0.0;
  F[17] = 1.0;
  F[33] = -0.0477 * std::sin(states[12]);
  F[49] = 0.0;
  F[65] = 0.0;
  F[81] = 0.0;
  F[97] = 0.0;
  F[113] = 0.0477 * std::cos(states[12]);
  F[129] = 0.0;
  F[145] = 0.0;
  F[161] = 0.0;
  F[177] = 0.0;
  F[193] = -0.0477 * states[7] * std::sin(states[12]) - 0.0477 * states[2] * std::
    cos(states[12]);
  F[209] = 0.0;
  F[225] = 0.0;
  F[241] = 0.0;
  for (i0 = 0; i0 < 16; i0++) {
    memcpy(&F[(i0 << 4) + 2], &dv1[i0 * 14], 14U * sizeof(double));
  }

  // predicted covariance estimate with linearised model
  eye(dv2);
  for (i0 = 0; i0 < 16; i0++) {
    for (c = 0; c < 16; c++) {
      b_F[i0 + (c << 4)] = 0.0;
      for (k = 0; k < 16; k++) {
        b_F[i0 + (c << 4)] += F[i0 + (k << 4)] * P[k + (c << 4)];
      }
    }

    for (c = 0; c < 16; c++) {
      d0 = 0.0;
      for (k = 0; k < 16; k++) {
        d0 += b_F[i0 + (k << 4)] * F[c + (k << 4)];
      }

      P_p[i0 + (c << 4)] = d0 + dv2[i0 + (c << 4)];
    }
  }

  // Update step
  // measurement residual with nonlinear model (H is always linear here)
  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    C_data[i0] = 0.0;
  }

  cr = 0;
  while (cr <= 0) {
    for (ic = 1; ic <= H_size_idx_0; ic++) {
      C_data[ic - 1] = 0.0;
    }

    cr = H_size_idx_0;
  }

  br = 0;
  cr = 0;
  while (cr <= 0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 16; ib++) {
      if (states_p[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic + 1 <= H_size_idx_0; ic++) {
          ia++;
          C_data[ic] += states_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 16;
    cr = H_size_idx_0;
  }

  for (i0 = 0; i0 < meas_size_idx_0; i0++) {
    meas_data[i0] -= C_data[i0];
  }

  // residual covariance with linear model
  for (i0 = 0; i0 < 16; i0++) {
    for (c = 0; c < H_size_idx_0; c++) {
      y_data[c + H_size_idx_0 * i0] = 0.0;
    }
  }

  c = H_size_idx_0 * 15;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    i0 = cr + H_size_idx_0;
    for (ic = cr; ic + 1 <= i0; ic++) {
      y_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 16; ib++) {
      if (P_p[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          y_data[ic] += P_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 16;
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 16; c++) {
      L_data[c + (i0 << 4)] = H_data[i0 + H_size_idx_0 * c];
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
    for (ib = br; ib + 1 <= br + 16; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          b_C_fs[ic] += L_data[ib] * y_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 16;
  }

  // optimal Kalman gain
  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 16; c++) {
      L_data[c + (i0 << 4)] = H_data[i0 + H_size_idx_0 * c];
    }
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    memset(&y_data[i0 << 4], 0, sizeof(double) << 4);
  }

  c = (H_size_idx_0 - 1) << 4;
  for (cr = 0; cr <= c; cr += 16) {
    for (ic = cr; ic + 1 <= cr + 16; ic++) {
      y_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 16) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 16; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 16; ic++) {
          ia++;
          y_data[ic] += L_data[ib] * P_p[ia];
        }
      }

      ar += 16;
    }

    br += 16;
  }

  C_size[0] = H_size_idx_0;
  C_size[1] = H_size_idx_0;
  k = H_size_idx_0 * H_size_idx_0;
  for (i0 = 0; i0 < k; i0++) {
    b_C_data[i0] = b_C_fs[i0] + R_data[i0];
  }

  inv(b_C_data, C_size, R_data, R_size);
  k = (signed char)R_size[1];
  for (i0 = 0; i0 < k; i0++) {
    memset(&L_data[i0 << 4], 0, sizeof(double) << 4);
  }

  c = (R_size[1] - 1) << 4;
  for (cr = 0; cr <= c; cr += 16) {
    for (ic = cr; ic + 1 <= cr + 16; ic++) {
      L_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 16) {
    ar = -1;
    i0 = br + H_size_idx_0;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (R_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 16; ic++) {
          ia++;
          L_data[ic] += R_data[ib] * y_data[ia];
        }
      }

      ar += 16;
    }

    br += H_size_idx_0;
  }

  // update states
  memset(&states[0], 0, sizeof(double) << 4);
  ar = -1;
  for (ib = 0; ib + 1 <= (signed char)R_size[1]; ib++) {
    if (meas_data[ib] != 0.0) {
      ia = ar;
      for (ic = 0; ic < 16; ic++) {
        ia++;
        states[ic] += meas_data[ib] * L_data[ia];
      }
    }

    ar += 16;
  }

  for (i0 = 0; i0 < 16; i0++) {
    states[i0] += states_p[i0];
  }

  // update covariance
  k = (signed char)R_size[1];
  memset(&F[0], 0, sizeof(double) << 8);
  for (cr = 0; cr <= 241; cr += 16) {
    for (ic = cr; ic + 1 <= cr + 16; ic++) {
      F[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 241; cr += 16) {
    ar = -1;
    i0 = br + k;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (H_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 16; ic++) {
          ia++;
          F[ic] += H_data[ib] * L_data[ia];
        }
      }

      ar += 16;
    }

    br += k;
  }

  c_eye(dv2);
  for (i0 = 0; i0 < 16; i0++) {
    for (c = 0; c < 16; c++) {
      b_F[c + (i0 << 4)] = dv2[c + (i0 << 4)] - F[c + (i0 << 4)];
    }
  }

  // Output of the function (for state feedback)
  for (k = 0; k < 16; k++) {
    for (i0 = 0; i0 < 16; i0++) {
      P[k + (i0 << 4)] = 0.0;
      for (c = 0; c < 16; c++) {
        P[k + (i0 << 4)] += b_F[k + (c << 4)] * P_p[c + (i0 << 4)];
      }
    }

    est[k] = states[k];
  }
}

//
// Arguments    : void
// Return Type  : void
//
void ekf_init()
{
  int k;

  //  Initialize the state vector the first time the function is used
  memset(&states[0], 0, sizeof(double) << 4);

  // [meas(1);meas(2);0;0;0;0;0;0;0;0;0;0;0;meas(6);0;0];
  //  Initialize the covariance vector the first time the function is used
  memset(&P[0], 0, sizeof(double) << 8);
  for (k = 0; k < 16; k++) {
    P[k + (k << 4)] = 1.0;
  }
}

//
// File trailer for ekf.cpp
//
// [EOF]
//
