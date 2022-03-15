//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: TransformTrajWrtWAM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>
#include "KnobOperationWAM.h"
#include "TransformTrajWrtWAM.h"
#include "quat2rotm.h"

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : const double PoseKnobWrtWAM[7]
//                const double CartTrajWrtKnob[123]
//                double CartTrajWristWrtWAM[123]
// Return Type  : void
//
void TransformTrajWrtWAM(const double PoseKnobWrtWAM[7], const double
  CartTrajWrtKnob[123], double CartTrajWristWrtWAM[123])
{
  double temp[9];
  int i5;
  double phi;
  double wamTwrist[16];
  static const signed char iv1[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  static const signed char iv2[3] = { 0, 0, 1 };

  int i6;
  double b_CartTrajWrtKnob[164];
  int i7;
  double b_wamTwrist[164];
  static const double b[9] = { 0.93969262078590843, 0.0, 0.34202014332566871,
    0.0, 1.0, 0.0, -0.34202014332566871, 0.0, 0.93969262078590843 };

  quat2rotm(*(double (*)[4])&PoseKnobWrtWAM[3], temp);
  for (i5 = 0; i5 < 16; i5++) {
    wamTwrist[i5] = iv1[i5];
  }

  phi = 3.1415926535897931 - rt_atan2d_snf(temp[7], temp[6]);

  //  angle of knob
  temp[0] = std::cos(phi);
  temp[3] = -std::sin(phi);
  temp[6] = 0.0;
  temp[1] = std::sin(phi);
  temp[4] = std::cos(phi);
  temp[7] = 0.0;
  for (i5 = 0; i5 < 3; i5++) {
    temp[2 + 3 * i5] = iv2[i5];
  }

  for (i5 = 0; i5 < 3; i5++) {
    for (i6 = 0; i6 < 3; i6++) {
      wamTwrist[i5 + (i6 << 2)] = 0.0;
      for (i7 = 0; i7 < 3; i7++) {
        wamTwrist[i5 + (i6 << 2)] += temp[i5 + 3 * i7] * b[i7 + 3 * i6];
      }
    }

    wamTwrist[12 + i5] = PoseKnobWrtWAM[i5];
  }

  for (i5 = 0; i5 < 41; i5++) {
    b_CartTrajWrtKnob[i5 << 2] = CartTrajWrtKnob[i5];
    b_CartTrajWrtKnob[1 + (i5 << 2)] = CartTrajWrtKnob[41 + i5];
    b_CartTrajWrtKnob[2 + (i5 << 2)] = CartTrajWrtKnob[82 + i5];
    b_CartTrajWrtKnob[3 + (i5 << 2)] = 1.0;
    for (i6 = 0; i6 < 4; i6++) {
      b_wamTwrist[i5 + 41 * i6] = 0.0;
      for (i7 = 0; i7 < 4; i7++) {
        b_wamTwrist[i5 + 41 * i6] += wamTwrist[i6 + (i7 << 2)] *
          b_CartTrajWrtKnob[i7 + (i5 << 2)];
      }
    }
  }

  for (i5 = 0; i5 < 3; i5++) {
    memcpy(&CartTrajWristWrtWAM[i5 * 41], &b_wamTwrist[i5 * 41], 41U * sizeof
           (double));
  }
}

//
// File trailer for TransformTrajWrtWAM.cpp
//
// [EOF]
//
