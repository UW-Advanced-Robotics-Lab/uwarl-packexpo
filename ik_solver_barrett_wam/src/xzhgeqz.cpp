//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzhgeqz.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xzhgeqz.h"
#include "xzlartg.h"
#include "sqrt.h"

// Function Definitions

//
// Arguments    : creal_T A[16]
//                int ilo
//                int ihi
//                creal_T Z[16]
//                int *info
//                creal_T alpha1[4]
//                creal_T beta1[4]
// Return Type  : void
//
void xzhgeqz(creal_T A[16], int ilo, int ihi, creal_T Z[16], int *info, creal_T
             alpha1[4], creal_T beta1[4])
{
  int i;
  double eshift_re;
  double eshift_im;
  creal_T ctemp;
  double anorm;
  double scale;
  double reAij;
  double sumsq;
  double b_atol;
  bool firstNonZero;
  int j;
  int jp1;
  double ascale;
  bool guard1 = false;
  double imAij;
  bool guard2 = false;
  int ifirst;
  int istart;
  int ilast;
  double temp2;
  int ilastm1;
  int iiter;
  bool goto60;
  bool goto70;
  bool goto90;
  int jiter;
  int exitg1;
  bool b_guard1 = false;
  bool guard3 = false;
  bool exitg2;
  creal_T b_ascale;
  creal_T shift;
  double ad22_re;
  double ad22_im;
  double t1_im;
  *info = 0;
  for (i = 0; i < 4; i++) {
    alpha1[i].re = 0.0;
    alpha1[i].im = 0.0;
    beta1[i].re = 1.0;
    beta1[i].im = 0.0;
  }

  eshift_re = 0.0;
  eshift_im = 0.0;
  ctemp.re = 0.0;
  ctemp.im = 0.0;
  anorm = 0.0;
  if (!(ilo > ihi)) {
    scale = 0.0;
    sumsq = 0.0;
    firstNonZero = true;
    for (j = ilo; j <= ihi; j++) {
      jp1 = j + 1;
      if (ihi < j + 1) {
        jp1 = ihi;
      }

      for (i = ilo; i <= jp1; i++) {
        reAij = A[(i + ((j - 1) << 2)) - 1].re;
        imAij = A[(i + ((j - 1) << 2)) - 1].im;
        if (reAij != 0.0) {
          anorm = std::abs(reAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }

        if (imAij != 0.0) {
          anorm = std::abs(imAij);
          if (firstNonZero) {
            sumsq = 1.0;
            scale = anorm;
            firstNonZero = false;
          } else if (scale < anorm) {
            temp2 = scale / anorm;
            sumsq = 1.0 + sumsq * temp2 * temp2;
            scale = anorm;
          } else {
            temp2 = anorm / scale;
            sumsq += temp2 * temp2;
          }
        }
      }
    }

    anorm = scale * std::sqrt(sumsq);
  }

  reAij = 2.2204460492503131E-16 * anorm;
  b_atol = 2.2250738585072014E-308;
  if (reAij > 2.2250738585072014E-308) {
    b_atol = reAij;
  }

  reAij = 2.2250738585072014E-308;
  if (anorm > 2.2250738585072014E-308) {
    reAij = anorm;
  }

  ascale = 1.0 / reAij;
  firstNonZero = true;
  for (j = ihi; j + 1 < 5; j++) {
    alpha1[j] = A[j + (j << 2)];
  }

  guard1 = false;
  guard2 = false;
  if (ihi >= ilo) {
    ifirst = ilo;
    istart = ilo;
    ilast = ihi - 1;
    ilastm1 = ihi - 2;
    iiter = 0;
    goto60 = false;
    goto70 = false;
    goto90 = false;
    jiter = 1;
    do {
      exitg1 = 0;
      if (jiter <= 30 * ((ihi - ilo) + 1)) {
        b_guard1 = false;
        if (ilast + 1 == ilo) {
          goto60 = true;
          b_guard1 = true;
        } else if (std::abs(A[ilast + (ilastm1 << 2)].re) + std::abs(A[ilast +
                    (ilastm1 << 2)].im) <= b_atol) {
          A[ilast + (ilastm1 << 2)].re = 0.0;
          A[ilast + (ilastm1 << 2)].im = 0.0;
          goto60 = true;
          b_guard1 = true;
        } else {
          j = ilastm1;
          guard3 = false;
          exitg2 = false;
          while ((!exitg2) && (j + 1 >= ilo)) {
            if (j + 1 == ilo) {
              guard3 = true;
              exitg2 = true;
            } else if (std::abs(A[j + ((j - 1) << 2)].re) + std::abs(A[j + ((j -
              1) << 2)].im) <= b_atol) {
              A[j + ((j - 1) << 2)].re = 0.0;
              A[j + ((j - 1) << 2)].im = 0.0;
              guard3 = true;
              exitg2 = true;
            } else {
              j--;
              guard3 = false;
            }
          }

          if (guard3) {
            ifirst = j + 1;
            goto70 = true;
          }

          if (goto70) {
            b_guard1 = true;
          } else {
            for (i = 0; i < 4; i++) {
              alpha1[i].re = rtNaN;
              alpha1[i].im = 0.0;
              beta1[i].re = rtNaN;
              beta1[i].im = 0.0;
            }

            for (jp1 = 0; jp1 < 16; jp1++) {
              Z[jp1].re = rtNaN;
              Z[jp1].im = 0.0;
            }

            *info = 1;
            exitg1 = 1;
          }
        }

        if (b_guard1) {
          if (goto60) {
            goto60 = false;
            alpha1[ilast] = A[ilast + (ilast << 2)];
            ilast = ilastm1;
            ilastm1--;
            if (ilast + 1 < ilo) {
              firstNonZero = false;
              guard2 = true;
              exitg1 = 1;
            } else {
              iiter = 0;
              eshift_re = 0.0;
              eshift_im = 0.0;
              jiter++;
            }
          } else {
            if (goto70) {
              goto70 = false;
              iiter++;
              if (iiter - iiter / 10 * 10 != 0) {
                anorm = ascale * A[ilastm1 + (ilastm1 << 2)].re;
                reAij = ascale * A[ilastm1 + (ilastm1 << 2)].im;
                if (reAij == 0.0) {
                  shift.re = anorm / 0.5;
                  shift.im = 0.0;
                } else if (anorm == 0.0) {
                  shift.re = 0.0;
                  shift.im = reAij / 0.5;
                } else {
                  shift.re = anorm / 0.5;
                  shift.im = reAij / 0.5;
                }

                anorm = ascale * A[ilast + (ilast << 2)].re;
                reAij = ascale * A[ilast + (ilast << 2)].im;
                if (reAij == 0.0) {
                  ad22_re = anorm / 0.5;
                  ad22_im = 0.0;
                } else if (anorm == 0.0) {
                  ad22_re = 0.0;
                  ad22_im = reAij / 0.5;
                } else {
                  ad22_re = anorm / 0.5;
                  ad22_im = reAij / 0.5;
                }

                temp2 = 0.5 * (shift.re + ad22_re);
                t1_im = 0.5 * (shift.im + ad22_im);
                anorm = ascale * A[ilastm1 + (ilast << 2)].re;
                reAij = ascale * A[ilastm1 + (ilast << 2)].im;
                if (reAij == 0.0) {
                  sumsq = anorm / 0.5;
                  imAij = 0.0;
                } else if (anorm == 0.0) {
                  sumsq = 0.0;
                  imAij = reAij / 0.5;
                } else {
                  sumsq = anorm / 0.5;
                  imAij = reAij / 0.5;
                }

                anorm = ascale * A[ilast + (ilastm1 << 2)].re;
                reAij = ascale * A[ilast + (ilastm1 << 2)].im;
                if (reAij == 0.0) {
                  scale = anorm / 0.5;
                  anorm = 0.0;
                } else if (anorm == 0.0) {
                  scale = 0.0;
                  anorm = reAij / 0.5;
                } else {
                  scale = anorm / 0.5;
                  anorm = reAij / 0.5;
                }

                reAij = shift.re * ad22_im + shift.im * ad22_re;
                shift.re = ((temp2 * temp2 - t1_im * t1_im) + (sumsq * scale -
                  imAij * anorm)) - (shift.re * ad22_re - shift.im * ad22_im);
                shift.im = ((temp2 * t1_im + t1_im * temp2) + (sumsq * anorm +
                  imAij * scale)) - reAij;
                c_sqrt(&shift);
                if ((temp2 - ad22_re) * shift.re + (t1_im - ad22_im) * shift.im <=
                    0.0) {
                  shift.re += temp2;
                  shift.im += t1_im;
                } else {
                  shift.re = temp2 - shift.re;
                  shift.im = t1_im - shift.im;
                }
              } else {
                anorm = ascale * A[ilast + (ilastm1 << 2)].re;
                reAij = ascale * A[ilast + (ilastm1 << 2)].im;
                if (reAij == 0.0) {
                  sumsq = anorm / 0.5;
                  imAij = 0.0;
                } else if (anorm == 0.0) {
                  sumsq = 0.0;
                  imAij = reAij / 0.5;
                } else {
                  sumsq = anorm / 0.5;
                  imAij = reAij / 0.5;
                }

                eshift_re += sumsq;
                eshift_im += imAij;
                shift.re = eshift_re;
                shift.im = eshift_im;
              }

              j = ilastm1;
              jp1 = ilastm1 + 1;
              exitg2 = false;
              while ((!exitg2) && (j + 1 > ifirst)) {
                istart = j + 1;
                ctemp.re = ascale * A[j + (j << 2)].re - shift.re * 0.5;
                ctemp.im = ascale * A[j + (j << 2)].im - shift.im * 0.5;
                anorm = std::abs(ctemp.re) + std::abs(ctemp.im);
                temp2 = ascale * (std::abs(A[jp1 + (j << 2)].re) + std::abs
                                  (A[jp1 + (j << 2)].im));
                reAij = anorm;
                if (temp2 > anorm) {
                  reAij = temp2;
                }

                if ((reAij < 1.0) && (reAij != 0.0)) {
                  anorm /= reAij;
                  temp2 /= reAij;
                }

                if ((std::abs(A[j + ((j - 1) << 2)].re) + std::abs(A[j + ((j - 1)
                       << 2)].im)) * temp2 <= anorm * b_atol) {
                  goto90 = true;
                  exitg2 = true;
                } else {
                  jp1 = j;
                  j--;
                }
              }

              if (!goto90) {
                istart = ifirst;
                ctemp.re = ascale * A[(ifirst + ((ifirst - 1) << 2)) - 1].re -
                  shift.re * 0.5;
                ctemp.im = ascale * A[(ifirst + ((ifirst - 1) << 2)) - 1].im -
                  shift.im * 0.5;
                goto90 = true;
              }
            }

            if (goto90) {
              goto90 = false;
              b_ascale.re = ascale * A[istart + ((istart - 1) << 2)].re;
              b_ascale.im = ascale * A[istart + ((istart - 1) << 2)].im;
              b_xzlartg(ctemp, b_ascale, &scale, &shift);
              j = istart;
              jp1 = istart - 2;
              while (j < ilast + 1) {
                if (j > istart) {
                  xzlartg(A[(j + (jp1 << 2)) - 1], A[j + (jp1 << 2)], &scale,
                          &shift, &A[(j + (jp1 << 2)) - 1]);
                  A[j + (jp1 << 2)].re = 0.0;
                  A[j + (jp1 << 2)].im = 0.0;
                }

                for (jp1 = j - 1; jp1 < 4; jp1++) {
                  ad22_re = scale * A[(j + (jp1 << 2)) - 1].re + (shift.re * A[j
                    + (jp1 << 2)].re - shift.im * A[j + (jp1 << 2)].im);
                  ad22_im = scale * A[(j + (jp1 << 2)) - 1].im + (shift.re * A[j
                    + (jp1 << 2)].im + shift.im * A[j + (jp1 << 2)].re);
                  anorm = A[(j + (jp1 << 2)) - 1].im;
                  reAij = A[(j + (jp1 << 2)) - 1].re;
                  A[j + (jp1 << 2)].re = scale * A[j + (jp1 << 2)].re -
                    (shift.re * A[(j + (jp1 << 2)) - 1].re + shift.im * A[(j +
                      (jp1 << 2)) - 1].im);
                  A[j + (jp1 << 2)].im = scale * A[j + (jp1 << 2)].im -
                    (shift.re * anorm - shift.im * reAij);
                  A[(j + (jp1 << 2)) - 1].re = ad22_re;
                  A[(j + (jp1 << 2)) - 1].im = ad22_im;
                }

                shift.re = -shift.re;
                shift.im = -shift.im;
                jp1 = j;
                if (ilast + 1 < j + 2) {
                  jp1 = ilast - 1;
                }

                for (i = 0; i < jp1 + 2; i++) {
                  ad22_re = scale * A[i + (j << 2)].re + (shift.re * A[i + ((j -
                    1) << 2)].re - shift.im * A[i + ((j - 1) << 2)].im);
                  ad22_im = scale * A[i + (j << 2)].im + (shift.re * A[i + ((j -
                    1) << 2)].im + shift.im * A[i + ((j - 1) << 2)].re);
                  anorm = A[i + (j << 2)].im;
                  reAij = A[i + (j << 2)].re;
                  A[i + ((j - 1) << 2)].re = scale * A[i + ((j - 1) << 2)].re -
                    (shift.re * A[i + (j << 2)].re + shift.im * A[i + (j << 2)].
                     im);
                  A[i + ((j - 1) << 2)].im = scale * A[i + ((j - 1) << 2)].im -
                    (shift.re * anorm - shift.im * reAij);
                  A[i + (j << 2)].re = ad22_re;
                  A[i + (j << 2)].im = ad22_im;
                }

                for (i = 0; i < 4; i++) {
                  ad22_re = scale * Z[i + (j << 2)].re + (shift.re * Z[i + ((j -
                    1) << 2)].re - shift.im * Z[i + ((j - 1) << 2)].im);
                  ad22_im = scale * Z[i + (j << 2)].im + (shift.re * Z[i + ((j -
                    1) << 2)].im + shift.im * Z[i + ((j - 1) << 2)].re);
                  anorm = Z[i + (j << 2)].im;
                  reAij = Z[i + (j << 2)].re;
                  Z[i + ((j - 1) << 2)].re = scale * Z[i + ((j - 1) << 2)].re -
                    (shift.re * Z[i + (j << 2)].re + shift.im * Z[i + (j << 2)].
                     im);
                  Z[i + ((j - 1) << 2)].im = scale * Z[i + ((j - 1) << 2)].im -
                    (shift.re * anorm - shift.im * reAij);
                  Z[i + (j << 2)].re = ad22_re;
                  Z[i + (j << 2)].im = ad22_im;
                }

                jp1 = j - 1;
                j++;
              }
            }

            jiter++;
          }
        }
      } else {
        guard2 = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  } else {
    guard1 = true;
  }

  if (guard2) {
    if (firstNonZero) {
      *info = ilast + 1;
      for (jp1 = 0; jp1 < ilast + 1; jp1++) {
        alpha1[jp1].re = rtNaN;
        alpha1[jp1].im = 0.0;
        beta1[jp1].re = rtNaN;
        beta1[jp1].im = 0.0;
      }

      for (jp1 = 0; jp1 < 16; jp1++) {
        Z[jp1].re = rtNaN;
        Z[jp1].im = 0.0;
      }
    } else {
      guard1 = true;
    }
  }

  if (guard1) {
    for (j = 0; j < ilo - 1; j++) {
      alpha1[j] = A[j + (j << 2)];
    }
  }
}

//
// File trailer for xzhgeqz.cpp
//
// [EOF]
//
