//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlascl.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "InverseKinematicsWAM.h"
#include "xzlascl.h"

// Function Definitions

//
// Arguments    : double cfrom
//                double cto
//                creal_T A[16]
// Return Type  : void
//
void xzlascl(double cfrom, double cto, creal_T A[16])
{
  double cfromc;
  double ctoc;
  bool notdone;
  double cfrom1;
  double cto1;
  double mul;
  int i5;
  cfromc = cfrom;
  ctoc = cto;
  notdone = true;
  while (notdone) {
    cfrom1 = cfromc * 2.0041683600089728E-292;
    cto1 = ctoc / 4.9896007738368E+291;
    if ((std::abs(cfrom1) > std::abs(ctoc)) && (ctoc != 0.0)) {
      mul = 2.0041683600089728E-292;
      cfromc = cfrom1;
    } else if (std::abs(cto1) > std::abs(cfromc)) {
      mul = 4.9896007738368E+291;
      ctoc = cto1;
    } else {
      mul = ctoc / cfromc;
      notdone = false;
    }

    for (i5 = 0; i5 < 16; i5++) {
      A[i5].re *= mul;
      A[i5].im *= mul;
    }
  }
}

//
// File trailer for xzlascl.cpp
//
// [EOF]
//
