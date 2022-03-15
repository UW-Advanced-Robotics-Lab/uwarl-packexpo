//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: schur.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "schur.h"
#include "xdlanv2.h"
#include "xdhseqr.h"
#include "xzlarf.h"
#include "xgehrd.h"
#include "InverseKinematicsWAM_rtwutil.h"

// Function Definitions

//
// Arguments    : const double A[16]
//                creal_T V[16]
//                creal_T T[16]
// Return Type  : void
//
void schur(const double A[16], creal_T V[16], creal_T T[16])
{
  bool p;
  int m;
  double b_A[16];
  int j;
  double tau[3];
  double Vr[16];
  int i;
  int itau;
  double work[4];
  int iaii;
  double r;
  double s;
  double t1_re;
  double t1_im;
  double mu1_im;
  double rt1i;
  double mu1_re;
  double rt2i;
  double cs;
  double sn;
  p = true;
  for (m = 0; m < 16; m++) {
    if (p && ((!rtIsInf(A[m])) && (!rtIsNaN(A[m])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (!p) {
    for (j = 0; j < 16; j++) {
      V[j].re = rtNaN;
      V[j].im = 0.0;
    }

    m = 3;
    for (j = 0; j < 2; j++) {
      for (i = m; i < 5; i++) {
        V[(i + (j << 2)) - 1].re = 0.0;
        V[(i + (j << 2)) - 1].im = 0.0;
      }

      m++;
    }

    for (j = 0; j < 16; j++) {
      T[j].re = rtNaN;
      T[j].im = 0.0;
    }
  } else {
    memcpy(&b_A[0], &A[0], sizeof(double) << 4);
    xgehrd(b_A, tau);
    memcpy(&Vr[0], &b_A[0], sizeof(double) << 4);
    for (j = 2; j >= 0; j--) {
      m = ((j + 1) << 2) - 1;
      for (i = 1; i <= j + 1; i++) {
        Vr[m + i] = 0.0;
      }

      for (i = j + 3; i < 5; i++) {
        Vr[m + i] = Vr[(m + i) - 4];
      }
    }

    for (i = 0; i < 4; i++) {
      Vr[i] = 0.0;
      work[i] = 0.0;
    }

    Vr[0] = 1.0;
    itau = 2;
    for (i = 2; i >= 0; i--) {
      iaii = (i + (i << 2)) + 5;
      if (i + 1 < 3) {
        Vr[iaii] = 1.0;
        xzlarf(3 - i, 2 - i, iaii + 1, tau[itau], Vr, iaii + 5, work);
        j = iaii - i;
        for (m = iaii + 1; m < j + 3; m++) {
          Vr[m] *= -tau[itau];
        }
      }

      Vr[iaii] = 1.0 - tau[itau];
      for (j = 1; j <= i; j++) {
        Vr[iaii - j] = 0.0;
      }

      itau--;
    }

    eml_dlahqr(b_A, Vr);
    b_A[3] = 0.0;
    for (j = 0; j < 16; j++) {
      T[j].re = b_A[j];
      T[j].im = 0.0;
      V[j].re = Vr[j];
      V[j].im = 0.0;
    }

    for (m = 2; m >= 0; m--) {
      if (b_A[(m + (m << 2)) + 1] != 0.0) {
        r = b_A[m + (m << 2)];
        s = b_A[m + ((m + 1) << 2)];
        t1_re = b_A[(m + (m << 2)) + 1];
        t1_im = b_A[(m + ((m + 1) << 2)) + 1];
        xdlanv2(&r, &s, &t1_re, &t1_im, &mu1_im, &rt1i, &mu1_re, &rt2i, &cs, &sn);
        mu1_re = mu1_im - b_A[(m + ((m + 1) << 2)) + 1];
        r = rt_hypotd_snf(rt_hypotd_snf(mu1_re, rt1i), b_A[(m + (m << 2)) + 1]);
        if (rt1i == 0.0) {
          mu1_re /= r;
          mu1_im = 0.0;
        } else if (mu1_re == 0.0) {
          mu1_re = 0.0;
          mu1_im = rt1i / r;
        } else {
          mu1_re /= r;
          mu1_im = rt1i / r;
        }

        s = b_A[(m + (m << 2)) + 1] / r;
        for (j = m; j + 1 < 5; j++) {
          t1_re = T[m + (j << 2)].re;
          t1_im = T[m + (j << 2)].im;
          r = T[m + (j << 2)].re;
          T[m + (j << 2)].re = (mu1_re * T[m + (j << 2)].re + mu1_im * T[m + (j <<
            2)].im) + s * T[(m + (j << 2)) + 1].re;
          T[m + (j << 2)].im = (mu1_re * T[m + (j << 2)].im - mu1_im * r) + s *
            T[(m + (j << 2)) + 1].im;
          r = mu1_re * T[(m + (j << 2)) + 1].im + mu1_im * T[(m + (j << 2)) + 1]
            .re;
          T[(m + (j << 2)) + 1].re = (mu1_re * T[(m + (j << 2)) + 1].re - mu1_im
            * T[(m + (j << 2)) + 1].im) - s * t1_re;
          T[(m + (j << 2)) + 1].im = r - s * t1_im;
        }

        for (i = 0; i < m + 2; i++) {
          t1_re = T[i + (m << 2)].re;
          t1_im = T[i + (m << 2)].im;
          r = mu1_re * T[i + (m << 2)].im + mu1_im * T[i + (m << 2)].re;
          T[i + (m << 2)].re = (mu1_re * T[i + (m << 2)].re - mu1_im * T[i + (m <<
            2)].im) + s * T[i + ((m + 1) << 2)].re;
          T[i + (m << 2)].im = r + s * T[i + ((m + 1) << 2)].im;
          r = T[i + ((m + 1) << 2)].re;
          T[i + ((m + 1) << 2)].re = (mu1_re * T[i + ((m + 1) << 2)].re + mu1_im
            * T[i + ((m + 1) << 2)].im) - s * t1_re;
          T[i + ((m + 1) << 2)].im = (mu1_re * T[i + ((m + 1) << 2)].im - mu1_im
            * r) - s * t1_im;
        }

        for (i = 0; i < 4; i++) {
          t1_re = V[i + (m << 2)].re;
          t1_im = V[i + (m << 2)].im;
          r = mu1_re * V[i + (m << 2)].im + mu1_im * V[i + (m << 2)].re;
          V[i + (m << 2)].re = (mu1_re * V[i + (m << 2)].re - mu1_im * V[i + (m <<
            2)].im) + s * V[i + ((m + 1) << 2)].re;
          V[i + (m << 2)].im = r + s * V[i + ((m + 1) << 2)].im;
          r = V[i + ((m + 1) << 2)].re;
          V[i + ((m + 1) << 2)].re = (mu1_re * V[i + ((m + 1) << 2)].re + mu1_im
            * V[i + ((m + 1) << 2)].im) - s * t1_re;
          V[i + ((m + 1) << 2)].im = (mu1_re * V[i + ((m + 1) << 2)].im - mu1_im
            * r) - s * t1_im;
        }

        T[(m + (m << 2)) + 1].re = 0.0;
        T[(m + (m << 2)) + 1].im = 0.0;
      }
    }
  }
}

//
// File trailer for schur.cpp
//
// [EOF]
//
