//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlarf.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xzlarf.h"
#include "xgerc.h"
#include "xgemv.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                double C[16]
//                int ic0
//                double work[4]
// Return Type  : void
//
void xzlarf(int m, int n, int iv0, double tau, double C[16], int ic0, double
            work[4])
{
  int lastv;
  int lastc;
  bool exitg2;
  int coltop;
  int ia;
  int exitg1;
  if (tau != 0.0) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C[lastc - 2] == 0.0)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = ic0 + ((lastc - 1) << 2);
      ia = coltop;
      do {
        exitg1 = 0;
        if (ia <= (coltop + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    xgemv(lastv, lastc, C, ic0, C, iv0, work);
    xgerc(lastv, lastc, -tau, iv0, work, C, ic0);
  }
}

//
// File trailer for xzlarf.cpp
//
// [EOF]
//
