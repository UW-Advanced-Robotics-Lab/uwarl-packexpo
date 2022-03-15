//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: rotm2quat.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "rotm2quat.h"
#include "schur.h"
#include "xzggev.h"

// Function Definitions

//
// Arguments    : const double R[9]
//                double quat[4]
// Return Type  : void
//
void rotm2quat(const double R[9], double quat[4])
{
  double K12;
  double K13;
  double K14;
  double K23;
  double K24;
  double K34;
  double K[16];
  bool p;
  int k;
  int j;
  bool exitg2;
  creal_T V[16];
  creal_T alpha1[4];
  creal_T At[16];
  int exitg1;
  double varargin_1[4];
  creal_T beta1[4];
  int coltop;
  K12 = R[3] + R[1];
  K13 = R[6] + R[2];
  K14 = R[5] - R[7];
  K23 = R[7] + R[5];
  K24 = R[6] - R[2];
  K34 = R[1] - R[3];
  K[0] = ((R[0] - R[4]) - R[8]) / 3.0;
  K[4] = K12 / 3.0;
  K[8] = K13 / 3.0;
  K[12] = K14 / 3.0;
  K[1] = K12 / 3.0;
  K[5] = ((R[4] - R[0]) - R[8]) / 3.0;
  K[9] = K23 / 3.0;
  K[13] = K24 / 3.0;
  K[2] = K13 / 3.0;
  K[6] = K23 / 3.0;
  K[10] = ((R[8] - R[0]) - R[4]) / 3.0;
  K[14] = K34 / 3.0;
  K[3] = K14 / 3.0;
  K[7] = K24 / 3.0;
  K[11] = K34 / 3.0;
  K[15] = ((R[0] + R[4]) + R[8]) / 3.0;
  p = true;
  for (k = 0; k < 16; k++) {
    if (p && ((!rtIsInf(K[k])) && (!rtIsNaN(K[k])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (!p) {
    for (k = 0; k < 16; k++) {
      V[k].re = rtNaN;
      V[k].im = 0.0;
    }

    for (k = 0; k < 4; k++) {
      alpha1[k].re = rtNaN;
      alpha1[k].im = 0.0;
    }
  } else {
    p = true;
    j = 0;
    exitg2 = false;
    while ((!exitg2) && (j < 4)) {
      k = 0;
      do {
        exitg1 = 0;
        if (k <= j) {
          if (!(K[k + (j << 2)] == K[j + (k << 2)])) {
            p = false;
            exitg1 = 1;
          } else {
            k++;
          }
        } else {
          j++;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }

    if (p) {
      schur(K, V, At);
      for (k = 0; k < 4; k++) {
        alpha1[k] = At[k + (k << 2)];
      }
    } else {
      for (k = 0; k < 16; k++) {
        At[k].re = K[k];
        At[k].im = 0.0;
      }

      xzggev(At, &k, alpha1, beta1, V);
      for (coltop = 0; coltop <= 13; coltop += 4) {
        K12 = 0.0;
        K13 = 3.3121686421112381E-170;
        for (k = coltop; k < coltop + 4; k++) {
          K14 = std::abs(V[k].re);
          if (K14 > K13) {
            K23 = K13 / K14;
            K12 = 1.0 + K12 * K23 * K23;
            K13 = K14;
          } else {
            K23 = K14 / K13;
            K12 += K23 * K23;
          }

          K14 = std::abs(V[k].im);
          if (K14 > K13) {
            K23 = K13 / K14;
            K12 = 1.0 + K12 * K23 * K23;
            K13 = K14;
          } else {
            K23 = K14 / K13;
            K12 += K23 * K23;
          }
        }

        K12 = K13 * std::sqrt(K12);
        for (j = coltop; j < coltop + 4; j++) {
          if (V[j].im == 0.0) {
            V[j].re /= K12;
            V[j].im = 0.0;
          } else if (V[j].re == 0.0) {
            V[j].re = 0.0;
            V[j].im /= K12;
          } else {
            V[j].re /= K12;
            V[j].im /= K12;
          }
        }
      }

      for (k = 0; k < 4; k++) {
        K23 = alpha1[k].re;
        if (beta1[k].im == 0.0) {
          if (alpha1[k].im == 0.0) {
            alpha1[k].re /= beta1[k].re;
            alpha1[k].im = 0.0;
          } else if (alpha1[k].re == 0.0) {
            alpha1[k].re = 0.0;
            alpha1[k].im /= beta1[k].re;
          } else {
            alpha1[k].re /= beta1[k].re;
            alpha1[k].im /= beta1[k].re;
          }
        } else if (beta1[k].re == 0.0) {
          if (alpha1[k].re == 0.0) {
            alpha1[k].re = alpha1[k].im / beta1[k].im;
            alpha1[k].im = 0.0;
          } else if (alpha1[k].im == 0.0) {
            alpha1[k].re = 0.0;
            alpha1[k].im = -(K23 / beta1[k].im);
          } else {
            alpha1[k].re = alpha1[k].im / beta1[k].im;
            alpha1[k].im = -(K23 / beta1[k].im);
          }
        } else {
          K14 = std::abs(beta1[k].re);
          K12 = std::abs(beta1[k].im);
          if (K14 > K12) {
            K12 = beta1[k].im / beta1[k].re;
            K13 = beta1[k].re + K12 * beta1[k].im;
            alpha1[k].re = (alpha1[k].re + K12 * alpha1[k].im) / K13;
            alpha1[k].im = (alpha1[k].im - K12 * K23) / K13;
          } else if (K12 == K14) {
            if (beta1[k].re > 0.0) {
              K12 = 0.5;
            } else {
              K12 = -0.5;
            }

            if (beta1[k].im > 0.0) {
              K13 = 0.5;
            } else {
              K13 = -0.5;
            }

            alpha1[k].re = (alpha1[k].re * K12 + alpha1[k].im * K13) / K14;
            alpha1[k].im = (alpha1[k].im * K12 - K23 * K13) / K14;
          } else {
            K12 = beta1[k].re / beta1[k].im;
            K13 = beta1[k].im + K12 * beta1[k].re;
            alpha1[k].re = (K12 * alpha1[k].re + alpha1[k].im) / K13;
            alpha1[k].im = (K12 * alpha1[k].im - K23) / K13;
          }
        }
      }
    }
  }

  for (k = 0; k < 4; k++) {
    varargin_1[k] = alpha1[k].re;
  }

  if (!rtIsNaN(varargin_1[0])) {
    j = 1;
  } else {
    j = 0;
    k = 2;
    exitg2 = false;
    while ((!exitg2) && (k < 5)) {
      if (!rtIsNaN(varargin_1[k - 1])) {
        j = k;
        exitg2 = true;
      } else {
        k++;
      }
    }
  }

  if (j != 0) {
    K12 = varargin_1[j - 1];
    k = j - 1;
    while (j + 1 < 5) {
      if (K12 < varargin_1[j]) {
        K12 = varargin_1[j];
        k = j;
      }

      j++;
    }

    j = k;
  }

  quat[0] = V[3 + (j << 2)].re;
  quat[1] = V[j << 2].re;
  quat[2] = V[1 + (j << 2)].re;
  quat[3] = V[2 + (j << 2)].re;
  if (V[3 + (j << 2)].re < 0.0) {
    for (k = 0; k < 4; k++) {
      quat[k] = -quat[k];
    }
  }
}

//
// File trailer for rotm2quat.cpp
//
// [EOF]
//
