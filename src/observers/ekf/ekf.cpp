//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ekf.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 29-Apr-2017 10:36:44
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "eye.h"
#include "inv.h"

// Variable Definitions
static double states[19];
static double P[361];

// Function Definitions

//
// Initialize constant matrices
// linear part of f function
//  % x dot ARX
// Arguments    : unsigned char fastslam_on
//                const double fastslam[6]
//                const double C_fs[36]
//                const double imu[2]
//                const double gyro[3]
//                double roll_ref
//                double pitch_ref
//                double yaw_ref
//                double thrust_ref
//                double est[19]
// Return Type  : void
//
void ekf(unsigned char fastslam_on, const double fastslam[6], const double C_fs
         [36], const double imu[2], const double gyro[3], double roll_ref,
         double pitch_ref, double yaw_ref, double thrust_ref, double est[19])
{
  double dv0[9];
  int i0;
  int c;
  double H_gyro[57];
  double cov[361];
  static const double dv1[361] = { 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5 };

  double b_imu[5];
  double meas_data[11];
  int meas_size_idx_0;
  double R_data[121];
  static const double dv2[25] = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1 };

  int H_size_idx_0;
  double b_C_fs[121];
  double H_imu[95];
  static const double b_H_imu[38] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1938,
    0.0, 0.1944, 0.0, -0.3103, 0.0, 0.0, 0.0, 0.0, 0.8608, 0.0, -0.4211, 0.0,
    -0.177, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0 };

  double H_data[209];
  static const double dv3[22] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  static const double dv4[33] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1 };

  double b_pitch_ref[4];
  double H_fs[209];
  static const double b_H_fs[114] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1938,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1944, 0.0, 0.0, 0.0, 0.0, 0.0, -0.3103, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8608,
    0.0, 0.0, 0.0, 0.0, 0.0, -0.4211, 0.0, 0.0, 0.0, 0.0, 0.0, -0.177, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0 };

  double states_p[19];
  double a[17];
  double F[361];
  double b_a[17];
  static const double c_a[289] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0907, 1.1869, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.7840670859538781, 0.0, 0.091,
    -0.3037, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -11.815513626834383, 0.0, -0.1452, 0.1289, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0293501048218046, 0.0, 0.0, -0.1045, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -0.42557651991614254, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4027, 0.261,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -15.874213836477988, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.197, 0.2374, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    9.40251572327044, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0828, 0.1581, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 6.5639412997903568, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0646, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1656184486373165,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9642, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.75052410901467626, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0473, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7913, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0 };

  static const double d_a[68] = { 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0628930817610067, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 18.046121593291407, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04379, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.9180293501048219, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };

  static const double dv5[323] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0907, 1.1869,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    4.7840670859538781, 0.0, 0.091, -0.3037, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -11.815513626834383, 0.0, -0.1452, 0.1289, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    7.0293501048218046, 0.0, 0.0, -0.1045, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.42557651991614254, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, -0.4027, 0.261, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -15.874213836477988, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.197, 0.2374, 0.0,
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.40251572327044, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0828, 0.1581, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 6.5639412997903568,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0646, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.1656184486373165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.9642, 0.0, 0.0, 0.0, 0.0, 0.0, -0.75052410901467626, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0473, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7913,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  double b_F[361];
  int k;
  int cr;
  double C_data[11];
  double d0;
  int br;
  int ic;
  double P_p[361];
  int ar;
  int ib;
  int ia;
  double L_data[209];
  int C_size[2];
  double b_C_data[121];
  int R_size[2];

  //  %y dot ARX
  //  %yaw ARX model
  //  % z model
  // %roll dot
  // %pitch dot
  // yaw dot
  // B matrix for prediction step
  // %x dot
  //  %y dot
  //  %yaw model
  //  %z
  // %roll dot
  // %pitch dot
  // pitch dot
  // linear h functions depending on measurements
  //  %x
  //  %y
  //  %z
  //  %roll
  //  %pitch
  // yaw
  //  %roll];
  // pitch
  eye(dv0);
  for (i0 = 0; i0 < 16; i0++) {
    for (c = 0; c < 3; c++) {
      H_gyro[c + 3 * i0] = 0.0;
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (c = 0; c < 3; c++) {
      H_gyro[c + 3 * (i0 + 16)] = dv0[c + 3 * i0];
    }
  }

  // weight matrix for states noise
  memcpy(&cov[0], &dv1[0], 361U * sizeof(double));
  cov[240] = 1.0;
  cov[360] = 1.0;

  // constant covariance noise matrix of imu and gyro
  // input vector
  // pi/2 here???
  // measurements and measurements covariance
  if (fastslam_on == 1) {
    for (i0 = 0; i0 < 6; i0++) {
      meas_data[i0] = fastslam[i0];
    }

    for (i0 = 0; i0 < 2; i0++) {
      meas_data[i0 + 6] = imu[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      meas_data[i0 + 8] = gyro[i0];
    }

    meas_size_idx_0 = 11;
    for (i0 = 0; i0 < 6; i0++) {
      for (c = 0; c < 6; c++) {
        b_C_fs[c + 11 * i0] = C_fs[c + 6 * i0];
      }
    }

    for (i0 = 0; i0 < 2; i0++) {
      for (c = 0; c < 6; c++) {
        b_C_fs[c + 11 * (i0 + 6)] = 0.0;
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (c = 0; c < 6; c++) {
        b_C_fs[c + 11 * (i0 + 8)] = 0.0;
      }
    }

    for (i0 = 0; i0 < 11; i0++) {
      for (c = 0; c < 2; c++) {
        b_C_fs[(c + 11 * i0) + 6] = dv3[c + (i0 << 1)];
      }

      for (c = 0; c < 3; c++) {
        b_C_fs[(c + 11 * i0) + 8] = dv4[c + 3 * i0];
      }

      memcpy(&R_data[i0 * 11], &b_C_fs[i0 * 11], 11U * sizeof(double));
    }

    H_size_idx_0 = 11;
    for (i0 = 0; i0 < 19; i0++) {
      for (c = 0; c < 6; c++) {
        H_fs[c + 11 * i0] = b_H_fs[c + 6 * i0];
      }

      for (c = 0; c < 2; c++) {
        H_fs[(c + 11 * i0) + 6] = b_H_imu[c + (i0 << 1)];
      }

      for (c = 0; c < 3; c++) {
        H_fs[(c + 11 * i0) + 8] = H_gyro[c + 3 * i0];
      }

      memcpy(&H_data[i0 * 11], &H_fs[i0 * 11], 11U * sizeof(double));
    }
  } else {
    for (i0 = 0; i0 < 2; i0++) {
      b_imu[i0] = imu[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_imu[i0 + 2] = gyro[i0];
    }

    meas_size_idx_0 = 5;
    for (i0 = 0; i0 < 5; i0++) {
      meas_data[i0] = b_imu[i0];
    }

    memcpy(&R_data[0], &dv2[0], 25U * sizeof(double));
    H_size_idx_0 = 5;
    for (i0 = 0; i0 < 19; i0++) {
      for (c = 0; c < 2; c++) {
        H_imu[c + 5 * i0] = b_H_imu[c + (i0 << 1)];
      }

      for (c = 0; c < 3; c++) {
        H_imu[(c + 5 * i0) + 2] = H_gyro[c + 3 * i0];
      }

      for (c = 0; c < 5; c++) {
        H_data[c + 5 * i0] = H_imu[c + 5 * i0];
      }
    }
  }

  //  Store the states from time to time.
  //  Store the covariance from time to time.
  // Prediction step
  // predicted states with nonlinear model
  // predicted x
  // predited y
  // predicted states constantly linear
  // predicted states vector
  b_pitch_ref[0] = pitch_ref;
  b_pitch_ref[1] = roll_ref;
  b_pitch_ref[2] = yaw_ref;
  b_pitch_ref[3] = thrust_ref - 0.587;
  states_p[0] = (states[0] + 0.0477 * std::cos(states[12]) * states[2]) - 0.0477
    * std::sin(states[12]) * states[7];
  states_p[1] = (states[1] + 0.0477 * std::cos(states[12]) * states[7]) + 0.0477
    * std::sin(states[12]) * states[2];
  for (i0 = 0; i0 < 17; i0++) {
    a[i0] = 0.0;
    for (c = 0; c < 17; c++) {
      a[i0] += c_a[i0 + 17 * c] * states[2 + c];
    }

    b_a[i0] = 0.0;
    for (c = 0; c < 4; c++) {
      b_a[i0] += d_a[i0 + 17 * c] * b_pitch_ref[c];
    }

    states_p[i0 + 2] = a[i0] + b_a[i0];
  }

  // linearised f nonlinear part (jacobian)
  //  %x lin
  // y lin
  F[0] = 1.0;
  F[19] = 0.0;
  F[38] = 0.0477 * std::cos(states[12]);
  F[57] = 0.0;
  F[76] = 0.0;
  F[95] = 0.0;
  F[114] = 0.0;
  F[133] = -0.0477 * std::sin(states[12]);
  F[152] = 0.0;
  F[171] = 0.0;
  F[190] = 0.0;
  F[209] = 0.0;
  F[228] = -0.0477 * states[2] * std::sin(states[12]) - 0.0477 * states[7] * std::
    cos(states[12]);
  F[247] = 0.0;
  F[266] = 0.0;
  F[285] = 0.0;
  F[1] = 0.0;
  F[20] = 1.0;
  F[39] = 0.0477 * std::sin(states[12]);
  F[58] = 0.0;
  F[77] = 0.0;
  F[96] = 0.0;
  F[115] = 0.0;
  F[134] = 0.0477 * std::cos(states[12]);
  F[153] = 0.0;
  F[172] = 0.0;
  F[191] = 0.0;
  F[210] = 0.0;
  F[229] = 0.0477 * states[7] * std::sin(states[12]) + 0.0477 * states[2] * std::
    cos(states[12]);
  F[248] = 0.0;
  F[267] = 0.0;
  F[286] = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (c = 0; c < 2; c++) {
      F[c + 19 * (i0 + 16)] = 0.0;
    }
  }

  for (i0 = 0; i0 < 19; i0++) {
    memcpy(&F[i0 * 19 + 2], &dv5[i0 * 17], 17U * sizeof(double));
  }

  // predicted covariance estimate with linearised model
  for (i0 = 0; i0 < 19; i0++) {
    for (c = 0; c < 19; c++) {
      b_F[i0 + 19 * c] = 0.0;
      for (k = 0; k < 19; k++) {
        b_F[i0 + 19 * c] += F[i0 + 19 * k] * P[k + 19 * c];
      }
    }

    for (c = 0; c < 19; c++) {
      d0 = 0.0;
      for (k = 0; k < 19; k++) {
        d0 += b_F[i0 + 19 * k] * F[c + 19 * k];
      }

      P_p[i0 + 19 * c] = d0 + cov[i0 + 19 * c];
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
    for (ib = br; ib + 1 <= br + 19; ib++) {
      if (states_p[ib] != 0.0) {
        ia = ar;
        for (ic = 0; ic + 1 <= H_size_idx_0; ic++) {
          ia++;
          C_data[ic] += states_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 19;
    cr = H_size_idx_0;
  }

  for (i0 = 0; i0 < meas_size_idx_0; i0++) {
    meas_data[i0] -= C_data[i0];
  }

  // residual covariance with linear model
  for (i0 = 0; i0 < 19; i0++) {
    for (c = 0; c < H_size_idx_0; c++) {
      H_fs[c + H_size_idx_0 * i0] = 0.0;
    }
  }

  c = H_size_idx_0 * 18;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    i0 = cr + H_size_idx_0;
    for (ic = cr; ic + 1 <= i0; ic++) {
      H_fs[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += H_size_idx_0) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 19; ib++) {
      if (P_p[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          H_fs[ic] += P_p[ib] * H_data[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 19;
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 19; c++) {
      L_data[c + 19 * i0] = H_data[i0 + H_size_idx_0 * c];
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
    for (ib = br; ib + 1 <= br + 19; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        i0 = cr + H_size_idx_0;
        for (ic = cr; ic + 1 <= i0; ic++) {
          ia++;
          b_C_fs[ic] += L_data[ib] * H_fs[ia];
        }
      }

      ar += H_size_idx_0;
    }

    br += 19;
  }

  // optimal Kalman gain
  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    for (c = 0; c < 19; c++) {
      L_data[c + 19 * i0] = H_data[i0 + H_size_idx_0 * c];
    }
  }

  for (i0 = 0; i0 < H_size_idx_0; i0++) {
    memset(&H_fs[i0 * 19], 0, 19U * sizeof(double));
  }

  c = 19 * (H_size_idx_0 - 1);
  for (cr = 0; cr <= c; cr += 19) {
    for (ic = cr; ic + 1 <= cr + 19; ic++) {
      H_fs[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 19) {
    ar = -1;
    for (ib = br; ib + 1 <= br + 19; ib++) {
      if (L_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 19; ic++) {
          ia++;
          H_fs[ic] += L_data[ib] * P_p[ia];
        }
      }

      ar += 19;
    }

    br += 19;
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
    memset(&L_data[i0 * 19], 0, 19U * sizeof(double));
  }

  c = 19 * (R_size[1] - 1);
  for (cr = 0; cr <= c; cr += 19) {
    for (ic = cr; ic + 1 <= cr + 19; ic++) {
      L_data[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= c; cr += 19) {
    ar = -1;
    i0 = br + H_size_idx_0;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (R_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 19; ic++) {
          ia++;
          L_data[ic] += R_data[ib] * H_fs[ia];
        }
      }

      ar += 19;
    }

    br += H_size_idx_0;
  }

  // update states
  memset(&states[0], 0, 19U * sizeof(double));
  ar = -1;
  for (ib = 0; ib + 1 <= (signed char)R_size[1]; ib++) {
    if (meas_data[ib] != 0.0) {
      ia = ar;
      for (ic = 0; ic < 19; ic++) {
        ia++;
        states[ic] += meas_data[ib] * L_data[ia];
      }
    }

    ar += 19;
  }

  for (i0 = 0; i0 < 19; i0++) {
    states[i0] += states_p[i0];
  }

  // update covariance
  k = (signed char)R_size[1];
  memset(&cov[0], 0, 361U * sizeof(double));
  for (cr = 0; cr <= 343; cr += 19) {
    for (ic = cr; ic + 1 <= cr + 19; ic++) {
      cov[ic] = 0.0;
    }
  }

  br = 0;
  for (cr = 0; cr <= 343; cr += 19) {
    ar = -1;
    i0 = br + k;
    for (ib = br; ib + 1 <= i0; ib++) {
      if (H_data[ib] != 0.0) {
        ia = ar;
        for (ic = cr; ic + 1 <= cr + 19; ic++) {
          ia++;
          cov[ic] += H_data[ib] * L_data[ia];
        }
      }

      ar += 19;
    }

    br += k;
  }

  b_eye(F);
  for (i0 = 0; i0 < 19; i0++) {
    for (c = 0; c < 19; c++) {
      b_F[c + 19 * i0] = F[c + 19 * i0] - cov[c + 19 * i0];
    }
  }

  // Output of the function (for state feedback)
  for (k = 0; k < 19; k++) {
    for (i0 = 0; i0 < 19; i0++) {
      P[k + 19 * i0] = 0.0;
      for (c = 0; c < 19; c++) {
        P[k + 19 * i0] += b_F[k + 19 * c] * P_p[c + 19 * i0];
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
  memset(&states[0], 0, 19U * sizeof(double));

  // [meas(1);meas(2);0;0;0;0;0;0;0;0;0;0;0;meas(6);0;0];
  //  Initialize the covariance vector the first time the function is used
  memset(&P[0], 0, 361U * sizeof(double));
  for (k = 0; k < 19; k++) {
    P[k + 19 * k] = 1.0;
  }
}

//
// File trailer for ekf.cpp
//
// [EOF]
//
