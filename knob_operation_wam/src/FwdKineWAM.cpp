//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: FwdKineWAM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//

// Include Files
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "KnobOperationWAM.h"
#include "FwdKineWAM.h"

// Function Definitions

//
// Arguments    : const double theta[7]
//                double Pn[24]
//                cell_wrap_0 baseTn[8]
//                cell_wrap_0 iprevTi[8]
//                cell_wrap_0 iTprevi[8]
// Return Type  : void
//
void FwdKineWAM(const double theta[7], double Pn[24], cell_wrap_0 baseTn[8],
                cell_wrap_0 iprevTi[8], cell_wrap_0 iTprevi[8])
{
  int k;
  double DH[32];
  static const double dv2[4] = { 0.0, 0.0, 0.1, 0.0 };

  double cq;
  double sq;
  double ca;
  int i2;
  double sa;
  int i3;
  double b_baseTn[16];
  int i4;
  static const signed char iv0[4] = { 0, 0, 0, 1 };

  double b_iprevTi[9];
  cell_wrap_0 b_iTprevi;
  memset(&baseTn[1].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[1].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[2].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[2].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[3].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[3].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[4].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[4].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[5].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[5].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[6].f1[0], 0, sizeof(double) << 4);
  for (k = 0; k < 4; k++) {
    baseTn[6].f1[k + (k << 2)] = 1.0;
  }

  memset(&baseTn[7].f1[0], 0, sizeof(double) << 4);

  //  WAM forward kinematics, 20190827, Jeongwoo Han
  //  Joint angle
  //  theta = [0; -pi/3; 0; pi/2; 0; pi/3; -pi/2;];
  //  theta = [0; 0; 0; 0; 0; 0; 0;];
  //  theta = theta_home;
  //  DH parameter
  DH[0] = 0.0;
  DH[8] = -1.5707963267948966;
  DH[16] = 0.0;
  DH[24] = theta[0];
  DH[1] = 0.0;
  DH[9] = 1.5707963267948966;
  DH[17] = 0.0;
  DH[25] = theta[1];
  DH[2] = 0.045;
  DH[10] = -1.5707963267948966;
  DH[18] = 0.55;
  DH[26] = theta[2];
  DH[3] = -0.045;
  DH[11] = 1.5707963267948966;
  DH[19] = 0.0;
  DH[27] = theta[3];
  DH[4] = 0.0;
  DH[12] = -1.5707963267948966;
  DH[20] = 0.3;
  DH[28] = theta[4];
  DH[5] = 0.0;
  DH[13] = 1.5707963267948966;
  DH[21] = 0.0;
  DH[29] = theta[5];
  DH[6] = 0.0;
  DH[14] = 0.0;
  DH[22] = 0.06;
  DH[30] = theta[6];
  for (k = 0; k < 4; k++) {
    baseTn[7].f1[k + (k << 2)] = 1.0;
    DH[7 + (k << 3)] = dv2[k];
  }

  //  Last row: J7 to tool
  for (k = 0; k < 8; k++) {
    cq = std::cos(DH[24 + k]);
    sq = std::sin(DH[24 + k]);
    ca = std::cos(DH[8 + k]);
    sa = std::sin(DH[8 + k]);
    iprevTi[k].f1[0] = cq;
    iprevTi[k].f1[4] = -sq * ca;
    iprevTi[k].f1[8] = sq * sa;
    iprevTi[k].f1[12] = DH[k] * cq;
    iprevTi[k].f1[1] = sq;
    iprevTi[k].f1[5] = cq * ca;
    iprevTi[k].f1[9] = -cq * sa;
    iprevTi[k].f1[13] = DH[k] * sq;
    iprevTi[k].f1[2] = 0.0;
    iprevTi[k].f1[6] = sa;
    iprevTi[k].f1[10] = ca;
    iprevTi[k].f1[14] = DH[16 + k];
    for (i2 = 0; i2 < 4; i2++) {
      iprevTi[k].f1[3 + (i2 << 2)] = iv0[i2];
    }

    memset(&iTprevi[k].f1[0], 0, sizeof(double) << 4);
    for (i2 = 0; i2 < 3; i2++) {
      for (i3 = 0; i3 < 3; i3++) {
        iTprevi[k].f1[i3 + (i2 << 2)] = iprevTi[k].f1[i2 + (i3 << 2)];
        b_iprevTi[i3 + 3 * i2] = -iprevTi[k].f1[i2 + (i3 << 2)];
      }
    }

    for (i2 = 0; i2 < 3; i2++) {
      iTprevi[k].f1[12 + i2] = 0.0;
      for (i3 = 0; i3 < 3; i3++) {
        b_iTprevi = iTprevi[k];
        b_iTprevi.f1[12 + i2] = iTprevi[k].f1[12 + i2] + b_iprevTi[i2 + 3 * i3] *
          iprevTi[k].f1[12 + i3];
        iTprevi[k] = b_iTprevi;
      }
    }

    for (i2 = 0; i2 < 4; i2++) {
      iTprevi[k].f1[3 + (i2 << 2)] = iv0[i2];
    }
  }

  memcpy(&baseTn[0].f1[0], &iprevTi[0].f1[0], sizeof(double) << 4);
  for (k = 0; k < 7; k++) {
    for (i2 = 0; i2 < 4; i2++) {
      for (i3 = 0; i3 < 4; i3++) {
        b_baseTn[i2 + (i3 << 2)] = 0.0;
        for (i4 = 0; i4 < 4; i4++) {
          b_baseTn[i2 + (i3 << 2)] += baseTn[k].f1[i2 + (i4 << 2)] * iprevTi[k +
            1].f1[i4 + (i3 << 2)];
        }
      }
    }

    for (i2 = 0; i2 < 4; i2++) {
      for (i3 = 0; i3 < 4; i3++) {
        baseTn[k + 1].f1[i3 + (i2 << 2)] = b_baseTn[i3 + (i2 << 2)];
      }
    }
  }

  for (k = 0; k < 8; k++) {
    for (i2 = 0; i2 < 3; i2++) {
      Pn[i2 + 3 * k] = baseTn[k].f1[12 + i2];
    }
  }
}

//
// File trailer for FwdKineWAM.cpp
//
// [EOF]
//
