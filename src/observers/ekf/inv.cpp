//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: inv.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 16-May-2017 14:57:32
//

// Include Files
#include "rt_nonfinite.h"
#include "ekf.h"
#include "inv.h"

// Function Declarations
static void invNxN(const double x_data[], const int x_size[2], double y_data[],
                   int y_size[2]);

// Function Definitions

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
static void invNxN(const double x_data[], const int x_size[2], double y_data[],
                   int y_size[2])
{
  int n;
  int yk;
  int i2;
  int minval;
  double b_x_data[49];
  int ipiv_data[7];
  int k;
  int u1;
  int j;
  int p_data[7];
  int mmj;
  int c;
  int ix;
  double smax;
  int kAcol;
  int jy;
  double s;
  int ijA;
  n = x_size[0];
  y_size[0] = x_size[0];
  y_size[1] = x_size[1];
  yk = x_size[0] * x_size[1];
  for (i2 = 0; i2 < yk; i2++) {
    y_data[i2] = 0.0;
  }

  yk = x_size[0] * x_size[1];
  for (i2 = 0; i2 < yk; i2++) {
    b_x_data[i2] = x_data[i2];
  }

  minval = x_size[0];
  ipiv_data[0] = 1;
  yk = 1;
  for (k = 2; k <= minval; k++) {
    yk++;
    ipiv_data[k - 1] = yk;
  }

  yk = x_size[0] - 1;
  u1 = x_size[0];
  if (yk < u1) {
    u1 = yk;
  }

  for (j = 0; j + 1 <= u1; j++) {
    mmj = n - j;
    c = j * (n + 1);
    if (mmj < 1) {
      yk = -1;
    } else {
      yk = 0;
      if (mmj > 1) {
        ix = c;
        smax = std::abs(b_x_data[c]);
        for (k = 2; k <= mmj; k++) {
          ix++;
          s = std::abs(b_x_data[ix]);
          if (s > smax) {
            yk = k - 1;
            smax = s;
          }
        }
      }
    }

    if (b_x_data[c + yk] != 0.0) {
      if (yk != 0) {
        ipiv_data[j] = (j + yk) + 1;
        ix = j;
        yk += j;
        for (k = 1; k <= n; k++) {
          smax = b_x_data[ix];
          b_x_data[ix] = b_x_data[yk];
          b_x_data[yk] = smax;
          ix += n;
          yk += n;
        }
      }

      i2 = c + mmj;
      for (jy = c + 1; jy + 1 <= i2; jy++) {
        b_x_data[jy] /= b_x_data[c];
      }
    }

    yk = n - j;
    kAcol = (c + n) + 1;
    jy = c + n;
    for (k = 1; k < yk; k++) {
      smax = b_x_data[jy];
      if (b_x_data[jy] != 0.0) {
        ix = c + 1;
        i2 = mmj + kAcol;
        for (ijA = kAcol; ijA + 1 < i2; ijA++) {
          b_x_data[ijA] += b_x_data[ix] * -smax;
          ix++;
        }
      }

      jy += n;
      kAcol += n;
    }
  }

  p_data[0] = 1;
  yk = 1;
  for (k = 2; k <= x_size[0]; k++) {
    yk++;
    p_data[k - 1] = yk;
  }

  for (k = 0; k < minval; k++) {
    if (ipiv_data[k] > 1 + k) {
      yk = p_data[ipiv_data[k] - 1];
      p_data[ipiv_data[k] - 1] = p_data[k];
      p_data[k] = yk;
    }
  }

  for (k = 0; k + 1 <= n; k++) {
    c = p_data[k] - 1;
    y_data[k + y_size[0] * (p_data[k] - 1)] = 1.0;
    for (j = k; j + 1 <= n; j++) {
      if (y_data[j + y_size[0] * c] != 0.0) {
        for (jy = j + 1; jy + 1 <= n; jy++) {
          y_data[jy + y_size[0] * c] -= y_data[j + y_size[0] * c] * b_x_data[jy
            + x_size[0] * j];
        }
      }
    }
  }

  for (j = 1; j <= n; j++) {
    yk = n * (j - 1);
    for (k = n - 1; k + 1 > 0; k--) {
      kAcol = n * k;
      if (y_data[k + yk] != 0.0) {
        y_data[k + yk] /= b_x_data[k + kAcol];
        for (jy = 0; jy + 1 <= k; jy++) {
          y_data[jy + yk] -= y_data[k + yk] * b_x_data[jy + kAcol];
        }
      }
    }
  }
}

//
// Arguments    : const double x_data[]
//                const int x_size[2]
//                double y_data[]
//                int y_size[2]
// Return Type  : void
//
void inv(const double x_data[], const int x_size[2], double y_data[], int
         y_size[2])
{
  invNxN(x_data, x_size, y_data, y_size);
}

//
// File trailer for inv.cpp
//
// [EOF]
//
