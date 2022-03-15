//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: InverseKinematicsWAM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//

// Include Files
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <string.h>
#include "InverseKinematicsWAM.h"
#include "norm.h"
#include "rotm2quat.h"
#include "acos.h"
#include "sqrt.h"
#include "cross.h"
#include "quat2rotm.h"

#include "ros/ros.h"

// Type Definitions
typedef struct {
  double f1[16];
} cell_wrap_0;

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
// Arguments    : double toolPoseDes[7]
//                const double JnCur[7]
//                double Theta_Jn[7]
//                double *
// Return Type  : void
//
void InverseKinematicsWAM(double toolPoseDes[7], const double JnCur[7], double
  Theta_Jn[7], double *success)
{
  int i;
  int i0;
  cell_wrap_0 iTprevi[8];
  static const signed char iv0[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0,
    0, 1 };

  cell_wrap_0 iprevTi[8];
  cell_wrap_0 baseTn[8];
  double DH[32];
  static const double dv0[32] = { 0.0, 0.0, 0.045, -0.045, 0.0, 0.0, 0.0, 0.0,
    -1.5707963267948966, 1.5707963267948966, -1.5707963267948966,
    1.5707963267948966, -1.5707963267948966, 1.5707963267948966, 0.0, 0.0, 0.0,
    0.0, 0.55, 0.0, 0.3, 0.0, 0.06, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  };

  static const double dv1[8] = { 0.0, 0.0, 0.045, -0.045, 0.0, 0.0, 0.0, 0.0 };

  double theta[8];
  double baseTtool[9];
  double b_baseTtool[16];
  int theta5_6_7_1_size_idx_1;
  double baseTwrist[16];
  static const signed char iv1[4] = { 1, 0, 0, 0 };

  static const signed char iv2[4] = { 0, 1, 0, 0 };

  static const signed char iv3[4] = { 0, 0, 0, 1 };

  double b_baseTwrist[16];
  int theta5_6_7_2_size_idx_0;
  double ca;
  double sa;
  double n7Tn6[16];
  double baseTn1[16];
  double baseTn2[16];
  double p26[3];
  double L3;
  double p6tool[3];
  double n_plane_for_p3[3];
  double A1;
  double B1;
  double C1;
  double F1;
  double F2;
  double F3;
  double d0;
  double pz_1;
  double p3_2[3];
  double p36_1[3];
  double p36_2[3];
  double c_baseTtool[2];
  double d_baseTtool[2];
  double b_p36_1[4];
  double b_iTprevi[4];
  cell_wrap_0 b_baseTn;
  double temp[4];
  double theta5_6_7_1_data[3];
  int theta5_6_7_2_size_idx_1;
  double theta5_6_7_2_data[3];
  double c_baseTn[7];
  double d_baseTn[7];

  //  Kinematics parameters
  //  PoseJ7Cur(1:3) = DH_scale*PoseJ7Cur(1:3);
  //  Transformation declaration
  for (i = 0; i < 7; i++) {
    Theta_Jn[i] = 0.0;
  }

  for (i0 = 0; i0 < 16; i0++) {
    iTprevi[0].f1[i0] = iv0[i0];
    iTprevi[1].f1[i0] = iv0[i0];
    iTprevi[2].f1[i0] = iv0[i0];
    iprevTi[3].f1[i0] = iv0[i0];
    baseTn[3].f1[i0] = iv0[i0];
    iprevTi[4].f1[i0] = iv0[i0];
    baseTn[4].f1[i0] = iv0[i0];
    iprevTi[5].f1[i0] = iv0[i0];
    baseTn[5].f1[i0] = iv0[i0];
    iprevTi[6].f1[i0] = iv0[i0];
    baseTn[6].f1[i0] = iv0[i0];
    iprevTi[7].f1[i0] = iv0[i0];
    baseTn[7].f1[i0] = iv0[i0];
  }

  //
  //  [Pn_cur, baseTn_cur, iprevTi_cur,iTprevi_cur] = FwdKineWAM(JnCur);
  //
  //  % Quauternion to Rotation matrix
  //  qx = PoseJ7Cur(4);qy = PoseJ7Cur(5);qz = PoseJ7Cur(6);qw = PoseJ7Cur(7);
  //  q = PoseJ7Cur(4:7)';
  //  R = quat2rotm(q);
  //
  //  baseT7 = eye(4);
  //  baseT7(1:3,1:3) = R;
  //  baseT7(1:3,4) = [PoseJ7Cur(1);PoseJ7Cur(2);PoseJ7Cur(3);];
  //  i7Ti6 = iTprevi_cur{7};
  //  i6Ti5 = iTprevi_cur{6};
  //  i5Ti4 = iTprevi_cur{5};
  //  i4Ti3 = iTprevi_cur{4};
  //
  //  % baseT3 = baseT7*iTprevi_cur{7}*iTprevi_cur{6}*iTprevi_cur{5}*iTprevi_cur{4}; 
  //  baseT3 = baseT7*i7Ti6*i6Ti5*i5Ti4*i5Ti4;
  //  baseT3
  //  p3_cur = baseT3(1:3,4);
  //  DH parameter
  //  DH = [0     -pi/2 0    theta(1);
  //        0      pi/2 0    theta(2);
  //        0.045 -pi/2 0.55 theta(3);
  //        -0.045 pi/2 0    theta(4);
  //        0     -pi/2 0.3  theta(5);
  //        0      pi/2 0    theta(6);
  //        0      0    0.06 theta(7);
  //        0      0    0.1  0;     ];  % Last row: J7 to tool
  memcpy(&DH[0], &dv0[0], sizeof(double) << 5);

  //  Last row: J7 to tool
  //  Get the current joint angles and corresponding poses
  //  theta = JnCur;
  //  [Pn,baseTn,iprevTi,iTprevi,DH] = FwdKineWAM(theta)
  for (i0 = 0; i0 < 8; i0++) {
    DH[i0] = dv1[i0];
    theta[i0] = DH[24 + i0];
  }

  //  setup for input of inverse kinematics
  //  From tool to wrist(J7), just translational offset and no rotation
  //  qx = toolPoseDes(4);qy = toolPoseDes(5);qz = toolPoseDes(6);qw = toolPoseDes(7); 
  //  s = 1/(qx^2+qy^2+qz^2+qw^2);
  //  R = [1-2*s*(qy^2+qz^2) 2*s*(qx*qy-qz*qw) 2*s*(qx*qz+qy*qw);
  //       2*s*(qx*qy+qz*qw) 1-2*s*(qx^2+qz^2)  2*s*(qy*qz-qx*qw);
  //       2*s*(qx*qz-qy*qw) 2*s*(qz*qx+qx*qw) 1-2*s*(qx^2+qy^2);];
  for (i0 = 0; i0 < 16; i0++) {
    b_baseTtool[i0] = iv0[i0];
  }

  quat2rotm(*(double (*)[4])&toolPoseDes[3], baseTtool);
  for (i0 = 0; i0 < 3; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
         theta5_6_7_1_size_idx_1++) {
      b_baseTtool[theta5_6_7_1_size_idx_1 + (i0 << 2)] =
        baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0];
    }
  }

  b_baseTtool[12] = toolPoseDes[0];
  b_baseTtool[13] = toolPoseDes[1];
  b_baseTtool[14] = toolPoseDes[2];
  baseTwrist[2] = 0.0;
  baseTwrist[6] = 0.0;
  baseTwrist[10] = 1.0;
  baseTwrist[14] = -DH[23];
  for (i0 = 0; i0 < 4; i0++) {
    baseTwrist[i0 << 2] = iv1[i0];
    baseTwrist[1 + (i0 << 2)] = iv2[i0];
    baseTwrist[3 + (i0 << 2)] = iv3[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
         theta5_6_7_1_size_idx_1++) {
      b_baseTwrist[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
      for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
           theta5_6_7_2_size_idx_0++) {
        b_baseTwrist[i0 + (theta5_6_7_1_size_idx_1 << 2)] += b_baseTtool[i0 +
          (theta5_6_7_2_size_idx_0 << 2)] * baseTwrist[theta5_6_7_2_size_idx_0 +
          (theta5_6_7_1_size_idx_1 << 2)];
      }
    }
  }

  //  baseTwrist = toolPoseDes*iTprevi{8};
  //  wristTbase = inv(baseTwrist);
  //  Get transform for 7->6 to get p6(origin of J6,i.e. wrist point). We only need point, so theta_7 can be arbitrary determined. So let it be 0.  
  //  Fwd transform
  theta[6] = 0.0;
  ca = std::cos(DH[14]);
  sa = std::sin(DH[14]);
  b_baseTtool[0] = 1.0;
  b_baseTtool[4] = -0.0 * ca;
  b_baseTtool[8] = 0.0 * sa;
  b_baseTtool[12] = DH[6];
  b_baseTtool[1] = 0.0;
  b_baseTtool[5] = ca;
  b_baseTtool[9] = -sa;
  b_baseTtool[13] = DH[6] * 0.0;
  b_baseTtool[2] = 0.0;
  b_baseTtool[6] = sa;
  b_baseTtool[10] = ca;
  b_baseTtool[14] = DH[22];
  for (i0 = 0; i0 < 4; i0++) {
    b_baseTtool[3 + (i0 << 2)] = iv3[i0];
  }

  //  Inv transform
  memset(&n7Tn6[0], 0, sizeof(double) << 4);
  n7Tn6[15] = 1.0;
  for (i0 = 0; i0 < 3; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
         theta5_6_7_1_size_idx_1++) {
      n7Tn6[theta5_6_7_1_size_idx_1 + (i0 << 2)] = b_baseTtool[i0 +
        (theta5_6_7_1_size_idx_1 << 2)];
      baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = -b_baseTtool[i0 +
        (theta5_6_7_1_size_idx_1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    n7Tn6[12 + i0] = 0.0;
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
         theta5_6_7_1_size_idx_1++) {
      n7Tn6[12 + i0] += baseTtool[i0 + 3 * theta5_6_7_1_size_idx_1] *
        b_baseTtool[12 + theta5_6_7_1_size_idx_1];
    }
  }

  //  Pose of J6 wrt base(J0), again we only need position, not orientation
  //  required position of J5 wrt base (origin of s
  //  To get the point 2 (Origin of J2) we need transfrom from base(0) to J2(2)
  //  Fwd transform
  ca = std::cos(DH[8]);
  sa = std::sin(DH[8]);
  baseTn1[0] = 1.0;
  baseTn1[4] = -0.0 * ca;
  baseTn1[8] = 0.0 * sa;
  baseTn1[12] = DH[0];
  baseTn1[1] = 0.0;
  baseTn1[5] = ca;
  baseTn1[9] = -sa;
  baseTn1[13] = DH[0] * 0.0;
  baseTn1[2] = 0.0;
  baseTn1[6] = sa;
  baseTn1[10] = ca;
  baseTn1[14] = DH[16];
  ca = std::cos(DH[9]);
  sa = std::sin(DH[9]);
  baseTwrist[0] = 1.0;
  baseTwrist[4] = -0.0 * ca;
  baseTwrist[8] = 0.0 * sa;
  baseTwrist[12] = DH[1];
  baseTwrist[1] = 0.0;
  baseTwrist[5] = ca;
  baseTwrist[9] = -sa;
  baseTwrist[13] = DH[1] * 0.0;
  baseTwrist[2] = 0.0;
  baseTwrist[6] = sa;
  baseTwrist[10] = ca;
  baseTwrist[14] = DH[17];
  for (i0 = 0; i0 < 4; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
         theta5_6_7_1_size_idx_1++) {
      b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
      for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
           theta5_6_7_2_size_idx_0++) {
        b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] += b_baseTwrist[i0 +
          (theta5_6_7_2_size_idx_0 << 2)] * n7Tn6[theta5_6_7_2_size_idx_0 +
          (theta5_6_7_1_size_idx_1 << 2)];
      }
    }

    baseTn1[3 + (i0 << 2)] = iv3[i0];
    baseTwrist[3 + (i0 << 2)] = iv3[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
         theta5_6_7_1_size_idx_1++) {
      baseTn2[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
      for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
           theta5_6_7_2_size_idx_0++) {
        baseTn2[i0 + (theta5_6_7_1_size_idx_1 << 2)] += baseTn1[i0 +
          (theta5_6_7_2_size_idx_0 << 2)] * baseTwrist[theta5_6_7_2_size_idx_0 +
          (theta5_6_7_1_size_idx_1 << 2)];
      }
    }
  }

  //  position of J2 wrt base
  //   p2 = [0;0;0];                          % position of J2 wrt base
  //  calculate the cross-sectional circle
  for (i0 = 0; i0 < 3; i0++) {
    p26[i0] = b_baseTtool[12 + i0] - baseTn2[12 + i0];
  }

  //  position vector from J2 to J5
  L3 = DH[18] * DH[18] + DH[2] * DH[2];
  b_sqrt(&L3);
  sa = DH[3] * DH[3] + DH[20] * DH[20];
  b_sqrt(&sa);

  //  position of J4(phisically, not analytically using DH), or p3 (regardless of orientation) 
  for (i0 = 0; i0 < 3; i0++) {
    p6tool[i0] = toolPoseDes[i0] - b_baseTtool[12 + i0];
  }

  cross(p26, p6tool, n_plane_for_p3);
  if (norm(n_plane_for_p3) == 0.0) {
    //  when p6tool is aligned with p26
    n_plane_for_p3[0] = p26[1];
    n_plane_for_p3[1] = -p26[0];
    n_plane_for_p3[2] = 0.0;
  }

  A1 = 2.0 * (baseTn2[12] - b_baseTtool[12]);
  B1 = 2.0 * (baseTn2[13] - b_baseTtool[13]);
  C1 = 2.0 * (baseTn2[14] - b_baseTtool[14]);
  ca = (((L3 * L3 - sa * sa) - (baseTn2[12] * baseTn2[12] - b_baseTtool[12] *
          b_baseTtool[12])) - (baseTn2[13] * baseTn2[13] - b_baseTtool[13] *
         b_baseTtool[13])) - (baseTn2[14] * baseTn2[14] - b_baseTtool[14] *
    b_baseTtool[14]);

  //  plane equation, vector pp2 is on the plane
  F1 = -(n_plane_for_p3[2] * A1 - C1 * n_plane_for_p3[0]) / (n_plane_for_p3[2] *
    B1 - C1 * n_plane_for_p3[1]);
  F2 = -(n_plane_for_p3[2] * ca - C1 * 0.0) / (n_plane_for_p3[2] * B1 - C1 *
    n_plane_for_p3[1]);
  F3 = -(A1 + -B1 * (n_plane_for_p3[2] * A1 - C1 * n_plane_for_p3[0]) /
         (n_plane_for_p3[2] * B1 - C1 * n_plane_for_p3[1])) / C1;
  A1 = -(-B1 * (n_plane_for_p3[2] * ca - C1 * 0.0) / (n_plane_for_p3[2] * B1 -
          C1 * n_plane_for_p3[1]) + ca) / C1;
  sa = (F1 * F2 + F3 * A1) / ((1.0 + F1 * F1) + F3 * F3);
  ca = ((L3 * L3 - F2 * F2) - A1 * A1) / ((1.0 + F1 * F1) + F3 * F3);
  d0 = ca + sa * sa;
  b_sqrt(&d0);
  C1 = d0 - sa;
  pz_1 = F3 * C1 + A1;
  d0 = ca + sa * sa;
  b_sqrt(&d0);
  B1 = -d0 - sa;
  sa = F3 * B1 + A1;
  n_plane_for_p3[0] = C1;
  n_plane_for_p3[1] = F1 * C1 + F2;
  n_plane_for_p3[2] = pz_1;
  p3_2[0] = B1;
  p3_2[1] = F1 * B1 + F2;
  p3_2[2] = sa;
  d0 = 0.0;
  A1 = 0.0;
  for (i0 = 0; i0 < 3; i0++) {
    L3 = b_baseTtool[12 + i0] - n_plane_for_p3[i0];
    ca = b_baseTtool[12 + i0] - p3_2[i0];
    d0 += p6tool[i0] * L3;
    A1 += p6tool[i0] * ca;
    p36_1[i0] = L3;
    p36_2[i0] = ca;
  }

  if (std::abs(d0) > std::abs(A1)) {
    for (i = 0; i < 3; i++) {
      p3_2[i] = n_plane_for_p3[i];
    }
  } else {
    d0 = 0.0;
    A1 = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      d0 += p6tool[i0] * p36_1[i0];
      A1 += p6tool[i0] * p36_2[i0];
    }

    if (!(std::abs(d0) < std::abs(A1))) {
      if (pz_1 - sa > 0.0) {
        for (i = 0; i < 3; i++) {
          p3_2[i] = n_plane_for_p3[i];
        }
      } else {
        if ((!(pz_1 - sa < 0.0)) && (C1 - B1 > 0.0)) {
          for (i = 0; i < 3; i++) {
            p3_2[i] = n_plane_for_p3[i];
          }
        }
      }
    }
  }

  cross(p3_2, p26, p36_1);
  A1 = 2.0 * (baseTn2[12] - p3_2[0]);
  B1 = 2.0 * (baseTn2[13] - p3_2[1]);
  C1 = 2.0 * (baseTn2[14] - p3_2[2]);
  ca = (((DH[18] * DH[18] - DH[2] * DH[2]) - (baseTn2[12] * baseTn2[12] - p3_2[0]
          * p3_2[0])) - (baseTn2[13] * baseTn2[13] - p3_2[1] * p3_2[1])) -
    (baseTn2[14] * baseTn2[14] - p3_2[2] * p3_2[2]);

  //  plane equation, vector pp2 is on the plane
  F1 = -(p36_1[2] * A1 - C1 * p36_1[0]) / (p36_1[2] * B1 - C1 * p36_1[1]);
  F2 = -(p36_1[2] * ca - C1 * 0.0) / (p36_1[2] * B1 - C1 * p36_1[1]);
  F3 = -(A1 + -B1 * (p36_1[2] * A1 - C1 * p36_1[0]) / (p36_1[2] * B1 - C1 *
          p36_1[1])) / C1;
  A1 = -(-B1 * (p36_1[2] * ca - C1 * 0.0) / (p36_1[2] * B1 - C1 * p36_1[1]) + ca)
    / C1;
  sa = (F1 * F2 + F3 * A1) / ((1.0 + F1 * F1) + F3 * F3);
  ca = ((DH[18] * DH[18] - F2 * F2) - A1 * A1) / ((1.0 + F1 * F1) + F3 * F3);
  d0 = ca + sa * sa;
  b_sqrt(&d0);
  C1 = d0 - sa;
  d0 = ca + sa * sa;
  b_sqrt(&d0);
  B1 = -d0 - sa;
  n_plane_for_p3[0] = C1;
  n_plane_for_p3[1] = F1 * C1 + F2;
  n_plane_for_p3[2] = F3 * C1 + A1;
  p36_1[0] = B1;
  p36_1[1] = F1 * B1 + F2;
  p36_1[2] = F3 * B1 + A1;
  for (i0 = 0; i0 < 2; i0++) {
    c_baseTtool[i0] = b_baseTtool[12 + i0] - n_plane_for_p3[i0];
    d_baseTtool[i0] = b_baseTtool[12 + i0] - p36_1[i0];
  }

  if (b_norm(c_baseTtool) > b_norm(d_baseTtool)) {
    for (i = 0; i < 3; i++) {
      p36_1[i] = n_plane_for_p3[i];
    }
  }

  //  Theta 1
  sa = rt_atan2d_snf(p36_1[1], p36_1[0]);
  if (sa < 0.0) {
    ca = sa + 3.1415926535897931;
  } else {
    ca = sa - 3.1415926535897931;
  }

  //  Check the range
  if (std::abs(sa - JnCur[0]) < std::abs(ca - JnCur[0])) {
    F3 = sa;
  } else {
    F3 = ca;
  }

  if ((ca < -2.6) || (ca > 2.6)) {
    F3 = sa;
  }

  if ((sa < -2.6) || (sa > 2.6)) {
    F3 = ca;
  }

  theta[0] = F3;

  //  Theta 2
  //      get the transformnation of J1 from J0
  B1 = std::cos(F3);
  C1 = std::sin(F3);
  ca = std::cos(DH[8]);
  sa = std::sin(DH[8]);
  iprevTi[0].f1[0] = B1;
  iprevTi[0].f1[4] = -C1 * ca;
  iprevTi[0].f1[8] = C1 * sa;
  iprevTi[0].f1[12] = DH[0] * B1;
  iprevTi[0].f1[1] = C1;
  iprevTi[0].f1[5] = B1 * ca;
  iprevTi[0].f1[9] = -B1 * sa;
  iprevTi[0].f1[13] = DH[0] * C1;
  iprevTi[0].f1[2] = 0.0;
  iprevTi[0].f1[6] = sa;
  iprevTi[0].f1[10] = ca;
  iprevTi[0].f1[14] = DH[16];
  for (i0 = 0; i0 < 4; i0++) {
    iprevTi[0].f1[3 + (i0 << 2)] = iv3[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
         theta5_6_7_1_size_idx_1++) {
      baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = -iprevTi[0].f1[i0 +
        (theta5_6_7_1_size_idx_1 << 2)];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    p6tool[i0] = 0.0;
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
         theta5_6_7_1_size_idx_1++) {
      iTprevi[0].f1[theta5_6_7_1_size_idx_1 + (i0 << 2)] = iprevTi[0].f1[i0 +
        (theta5_6_7_1_size_idx_1 << 2)];
      p6tool[i0] += baseTtool[i0 + 3 * theta5_6_7_1_size_idx_1] * iprevTi[0].f1
        [12 + theta5_6_7_1_size_idx_1];
    }

    iTprevi[0].f1[12 + i0] = p6tool[i0];
  }

  for (i0 = 0; i0 < 4; i0++) {
    iTprevi[0].f1[3 + (i0 << 2)] = iv3[i0];
  }

  //  Transformation from base to j1
  memcpy(&baseTn[0].f1[0], &iprevTi[0].f1[0], sizeof(double) << 4);
  for (i0 = 0; i0 < 3; i0++) {
    b_p36_1[i0] = p36_1[i0];
  }

  b_p36_1[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    b_iTprevi[i0] = 0.0;
    for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
         theta5_6_7_1_size_idx_1++) {
      b_iTprevi[i0] += iTprevi[0].f1[i0 + (theta5_6_7_1_size_idx_1 << 2)] *
        b_p36_1[theta5_6_7_1_size_idx_1];
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    n_plane_for_p3[i0] = b_iTprevi[i0];
  }

  A1 = rt_atan2d_snf(n_plane_for_p3[0], -n_plane_for_p3[1]);
  if ((A1 < -2.0) || (A1 > 2.0)) {
    //     disp("Theta 2 is out of range. Terminated");
    *success = -2.0;
  } else {
    theta[1] = A1;

    //  Theta 3
    //  get the transformnation of J2 from J1
    B1 = std::cos(A1);
    C1 = std::sin(A1);
    ca = std::cos(DH[9]);
    sa = std::sin(DH[9]);
    iprevTi[1].f1[0] = B1;
    iprevTi[1].f1[4] = -C1 * ca;
    iprevTi[1].f1[8] = C1 * sa;
    iprevTi[1].f1[12] = DH[1] * B1;
    iprevTi[1].f1[1] = C1;
    iprevTi[1].f1[5] = B1 * ca;
    iprevTi[1].f1[9] = -B1 * sa;
    iprevTi[1].f1[13] = DH[1] * C1;
    iprevTi[1].f1[2] = 0.0;
    iprevTi[1].f1[6] = sa;
    iprevTi[1].f1[10] = ca;
    iprevTi[1].f1[14] = DH[17];
    for (i0 = 0; i0 < 4; i0++) {
      iprevTi[1].f1[3 + (i0 << 2)] = iv3[i0];
    }

    for (i0 = 0; i0 < 3; i0++) {
      for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
           theta5_6_7_1_size_idx_1++) {
        baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = -iprevTi[1].f1[i0 +
          (theta5_6_7_1_size_idx_1 << 2)];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      p6tool[i0] = 0.0;
      for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
           theta5_6_7_1_size_idx_1++) {
        iTprevi[1].f1[theta5_6_7_1_size_idx_1 + (i0 << 2)] = iprevTi[1].f1[i0 +
          (theta5_6_7_1_size_idx_1 << 2)];
        p6tool[i0] += baseTtool[i0 + 3 * theta5_6_7_1_size_idx_1] * iprevTi[1].
          f1[12 + theta5_6_7_1_size_idx_1];
      }

      iTprevi[1].f1[12 + i0] = p6tool[i0];
    }

    //  Transformation from base to j2
    for (i0 = 0; i0 < 4; i0++) {
      iTprevi[1].f1[3 + (i0 << 2)] = iv3[i0];
      for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
           theta5_6_7_1_size_idx_1++) {
        baseTn[1].f1[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
        b_baseTn = baseTn[1];
        for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
             theta5_6_7_2_size_idx_0++) {
          b_baseTn.f1[i0 + (theta5_6_7_1_size_idx_1 << 2)] += iprevTi[0].f1[i0 +
            (theta5_6_7_2_size_idx_0 << 2)] * iprevTi[1]
            .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
          baseTn[1] = b_baseTn;
        }
      }
    }

    for (i0 = 0; i0 < 4; i0++) {
      for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
           theta5_6_7_1_size_idx_1++) {
        b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
        for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
             theta5_6_7_2_size_idx_0++) {
          b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] += iTprevi[1].f1[i0 +
            (theta5_6_7_2_size_idx_0 << 2)] * iTprevi[0]
            .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
        }
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      b_p36_1[i0] = p3_2[i0] - p36_1[i0];
    }

    b_p36_1[3] = 1.0;
    for (i0 = 0; i0 < 4; i0++) {
      b_iTprevi[i0] = 0.0;
      for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
           theta5_6_7_1_size_idx_1++) {
        b_iTprevi[i0] += b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] *
          b_p36_1[theta5_6_7_1_size_idx_1];
      }
    }

    for (i0 = 0; i0 < 3; i0++) {
      n_plane_for_p3[i0] = b_iTprevi[i0];
    }

    L3 = rt_atan2d_snf(n_plane_for_p3[1], n_plane_for_p3[0]);
    if ((L3 < -2.8) || (L3 > 2.8)) {
      //     disp("Theta 3 is out of range. Terminated");
      *success = -3.0;
    } else {
      theta[2] = L3;

      //  Theta 4
      B1 = std::cos(L3);
      C1 = std::sin(L3);
      ca = std::cos(DH[10]);
      sa = std::sin(DH[10]);
      iprevTi[2].f1[0] = B1;
      iprevTi[2].f1[4] = -C1 * ca;
      iprevTi[2].f1[8] = C1 * sa;
      iprevTi[2].f1[12] = DH[2] * B1;
      iprevTi[2].f1[1] = C1;
      iprevTi[2].f1[5] = B1 * ca;
      iprevTi[2].f1[9] = -B1 * sa;
      iprevTi[2].f1[13] = DH[2] * C1;
      iprevTi[2].f1[2] = 0.0;
      iprevTi[2].f1[6] = sa;
      iprevTi[2].f1[10] = ca;
      iprevTi[2].f1[14] = DH[18];
      for (i0 = 0; i0 < 4; i0++) {
        iprevTi[2].f1[3 + (i0 << 2)] = iv3[i0];
      }

      for (i0 = 0; i0 < 3; i0++) {
        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
             theta5_6_7_1_size_idx_1++) {
          baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = -iprevTi[2].f1[i0 +
            (theta5_6_7_1_size_idx_1 << 2)];
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        p6tool[i0] = 0.0;
        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
             theta5_6_7_1_size_idx_1++) {
          iTprevi[2].f1[theta5_6_7_1_size_idx_1 + (i0 << 2)] = iprevTi[2].f1[i0
            + (theta5_6_7_1_size_idx_1 << 2)];
          p6tool[i0] += baseTtool[i0 + 3 * theta5_6_7_1_size_idx_1] * iprevTi[2]
            .f1[12 + theta5_6_7_1_size_idx_1];
        }

        iTprevi[2].f1[12 + i0] = p6tool[i0];
      }

      //  Transformation from base to j3
      for (i0 = 0; i0 < 4; i0++) {
        iTprevi[2].f1[3 + (i0 << 2)] = iv3[i0];
        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
             theta5_6_7_1_size_idx_1++) {
          baseTn[2].f1[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
          for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
               theta5_6_7_2_size_idx_0++) {
            baseTn[2].f1[i0 + (theta5_6_7_1_size_idx_1 << 2)] += baseTn[1].f1[i0
              + (theta5_6_7_2_size_idx_0 << 2)] * iprevTi[2]
              .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
          }
        }
      }

      for (i0 = 0; i0 < 4; i0++) {
        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
             theta5_6_7_1_size_idx_1++) {
          b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
          baseTwrist[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
          for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
               theta5_6_7_2_size_idx_0++) {
            b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] += iTprevi[2].f1[i0
              + (theta5_6_7_2_size_idx_0 << 2)] * iTprevi[1]
              .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
            baseTwrist[i0 + (theta5_6_7_1_size_idx_1 << 2)] += b_baseTwrist[i0 +
              (theta5_6_7_2_size_idx_0 << 2)] * n7Tn6[theta5_6_7_2_size_idx_0 +
              (theta5_6_7_1_size_idx_1 << 2)];
          }
        }

        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
             theta5_6_7_1_size_idx_1++) {
          baseTn2[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
          for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
               theta5_6_7_2_size_idx_0++) {
            baseTn2[i0 + (theta5_6_7_1_size_idx_1 << 2)] += b_baseTtool[i0 +
              (theta5_6_7_2_size_idx_0 << 2)] * iTprevi[0]
              .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
          }
        }
      }

      for (i0 = 0; i0 < 3; i0++) {
        b_p36_1[i0] = baseTwrist[12 + i0];
      }

      b_p36_1[3] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        temp[i0] = 0.0;
        for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
             theta5_6_7_1_size_idx_1++) {
          temp[i0] += baseTn2[i0 + (theta5_6_7_1_size_idx_1 << 2)] *
            b_p36_1[theta5_6_7_1_size_idx_1];
        }
      }

      sa = rt_atan2d_snf(temp[0], -temp[1]) + std::atan(DH[2] / DH[20]);

      //   theta_4 = pi - acos((L3^2+L5^2-p26'*p26)/(2*L3*L5))+ atan(a(3)/d(3)) + atan(a(3)/d(5));  
      if ((sa < -0.9) || (sa > 3.1)) {
        //     disp("Theta 4 is out of range. Terminated");
        *success = -4.0;
      } else {
        theta[3] = sa;
        Theta_Jn[0] = F3;
        Theta_Jn[1] = A1;
        Theta_Jn[2] = L3;
        Theta_Jn[3] = sa;

        //  For wrist, J5~7
        //  Get the transform from base to J4 (elbow)
        for (i = 0; i < 4; i++) {
          B1 = std::cos(theta[i]);
          C1 = std::sin(theta[i]);
          ca = std::cos(DH[8 + i]);
          sa = std::sin(DH[8 + i]);
          iprevTi[i].f1[0] = B1;
          iprevTi[i].f1[4] = -C1 * ca;
          iprevTi[i].f1[8] = C1 * sa;
          iprevTi[i].f1[12] = DH[i] * B1;
          iprevTi[i].f1[1] = C1;
          iprevTi[i].f1[5] = B1 * ca;
          iprevTi[i].f1[9] = -B1 * sa;
          iprevTi[i].f1[13] = DH[i] * C1;
          iprevTi[i].f1[2] = 0.0;
          iprevTi[i].f1[6] = sa;
          iprevTi[i].f1[10] = ca;
          iprevTi[i].f1[14] = DH[16 + i];
          for (i0 = 0; i0 < 4; i0++) {
            iprevTi[i].f1[3 + (i0 << 2)] = iv3[i0];
          }

          if (1 + i == 1) {
            for (i0 = 0; i0 < 16; i0++) {
              baseTn[0].f1[i0] = iprevTi[0].f1[i0];
            }
          } else {
            for (i0 = 0; i0 < 4; i0++) {
              for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
                   theta5_6_7_1_size_idx_1++) {
                b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
                for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
                     theta5_6_7_2_size_idx_0++) {
                  b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] += baseTn[i -
                    1].f1[i0 + (theta5_6_7_2_size_idx_0 << 2)] * iprevTi[i]
                    .f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
                }
              }
            }

            for (i0 = 0; i0 < 4; i0++) {
              for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
                   theta5_6_7_1_size_idx_1++) {
                baseTn[i].f1[theta5_6_7_1_size_idx_1 + (i0 << 2)] =
                  b_baseTtool[theta5_6_7_1_size_idx_1 + (i0 << 2)];
              }
            }
          }
        }

        //  transform matrix from elbow(J4) to base. i.e. inv of T from base to
        //  elbow
        memset(&b_baseTtool[0], 0, sizeof(double) << 4);
        for (i0 = 0; i0 < 3; i0++) {
          for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
               theta5_6_7_1_size_idx_1++) {
            b_baseTtool[theta5_6_7_1_size_idx_1 + (i0 << 2)] = baseTn[3].f1[i0 +
              (theta5_6_7_1_size_idx_1 << 2)];
            baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = -baseTn[3].f1[i0 +
              (theta5_6_7_1_size_idx_1 << 2)];
          }
        }

        for (i0 = 0; i0 < 3; i0++) {
          b_baseTtool[12 + i0] = 0.0;
          for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
               theta5_6_7_1_size_idx_1++) {
            b_baseTtool[12 + i0] += baseTtool[i0 + 3 * theta5_6_7_1_size_idx_1] *
              baseTn[3].f1[12 + theta5_6_7_1_size_idx_1];
          }
        }

        for (i0 = 0; i0 < 4; i0++) {
          b_baseTtool[3 + (i0 << 2)] = iv3[i0];
        }

        //  S is the transformation from 4 to 7 (i.e. frame 7 viewed from 4)
        for (i0 = 0; i0 < 4; i0++) {
          for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
               theta5_6_7_1_size_idx_1++) {
            baseTn1[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
            for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
                 theta5_6_7_2_size_idx_0++) {
              baseTn1[i0 + (theta5_6_7_1_size_idx_1 << 2)] += b_baseTtool[i0 +
                (theta5_6_7_2_size_idx_0 << 2)] *
                b_baseTwrist[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 <<
                2)];
            }
          }
        }

        //  Find theta5,6,7
        //  theta6 has two solutions.
        ca = baseTn1[10];
        b_acos(&ca);
        sa = rt_atan2d_snf(baseTn1[6], -baseTn1[2]);
        B1 = rt_atan2d_snf(baseTn1[9], baseTn1[8]);
        p6tool[0] = B1;
        p6tool[1] = ca;
        p6tool[2] = sa;
        i = 1;
        theta5_6_7_1_size_idx_1 = 3;
        for (i0 = 0; i0 < 3; i0++) {
          theta5_6_7_1_data[i0] = p6tool[i0];
        }

        d0 = baseTn1[10];
        b_acos(&d0);
        C1 = rt_atan2d_snf(-baseTn1[6], baseTn1[2]);
        A1 = rt_atan2d_snf(-baseTn1[9], -baseTn1[8]);
        p6tool[0] = A1;
        p6tool[1] = -d0;
        p6tool[2] = C1;
        theta5_6_7_2_size_idx_0 = 1;
        theta5_6_7_2_size_idx_1 = 3;
        for (i0 = 0; i0 < 3; i0++) {
          theta5_6_7_2_data[i0] = p6tool[i0];
        }

        //  Check the range of operation
        if ((B1 < -4.76) || (B1 > 1.24) || (ca < -1.6) || (ca > 1.6) || (sa <
             -3.0) || (sa > 3.0)) {
          i = 0;
          theta5_6_7_1_size_idx_1 = 0;
        }

        if ((A1 < -4.76) || (A1 > 1.24) || (-d0 < -1.6) || (-d0 > 1.6) || (C1 <
             -3.0) || (C1 > 3.0)) {
          theta5_6_7_2_size_idx_0 = 0;
          theta5_6_7_2_size_idx_1 = 0;
        }

        //  Select best set of wrist angles (5,6,7)
        if (((i == 0) || (theta5_6_7_1_size_idx_1 == 0)) &&
            ((theta5_6_7_2_size_idx_0 == 0) || (theta5_6_7_2_size_idx_1 == 0)))
        {
          //      disp('No feasible theta_5_6_7 for wrist from current elbow point. Terminated'); 
          *success = -5.0;
        } else {
          if (!((i == 0) || (theta5_6_7_1_size_idx_1 == 0))) {
            if ((theta5_6_7_2_size_idx_0 == 0) || (theta5_6_7_2_size_idx_1 == 0))
            {
              memcpy(&theta5_6_7_2_data[0], &theta5_6_7_1_data[0], (unsigned int)
                     (theta5_6_7_1_size_idx_1 * (int)sizeof(double)));
            } else {
              p6tool[0] = JnCur[4];
              p6tool[1] = JnCur[5];
              p6tool[2] = JnCur[6];
              p26[0] = JnCur[4];
              p26[1] = JnCur[5];
              p26[2] = JnCur[6];
              d0 = 0.0;
              A1 = 0.0;
              for (i0 = 0; i0 < 3; i0++) {
                sa = theta5_6_7_1_data[i0] - p6tool[i0];
                L3 = theta5_6_7_2_data[i0] - p26[i0];
                d0 += sa * sa;
                A1 += L3 * L3;
              }

              if (d0 <= A1) {
                memcpy(&theta5_6_7_2_data[0], &theta5_6_7_1_data[0], (unsigned
                        int)(theta5_6_7_1_size_idx_1 * (int)sizeof(double)));
              }
            }
          }

          theta[4] = theta5_6_7_2_data[0];
          theta[5] = theta5_6_7_2_data[1];
          theta[6] = theta5_6_7_2_data[2];
          Theta_Jn[4] = theta5_6_7_2_data[0];
          Theta_Jn[5] = theta5_6_7_2_data[1];
          Theta_Jn[6] = theta5_6_7_2_data[2];
          for (i = 0; i < 4; i++) {
            B1 = std::cos(theta[i + 4]);
            C1 = std::sin(theta[i + 4]);
            ca = std::cos(DH[i + 12]);
            sa = std::sin(DH[i + 12]);
            iprevTi[i + 4].f1[0] = B1;
            iprevTi[i + 4].f1[4] = -C1 * ca;
            iprevTi[i + 4].f1[8] = C1 * sa;
            iprevTi[i + 4].f1[12] = DH[i + 4] * B1;
            iprevTi[i + 4].f1[1] = C1;
            iprevTi[i + 4].f1[5] = B1 * ca;
            iprevTi[i + 4].f1[9] = -B1 * sa;
            iprevTi[i + 4].f1[13] = DH[i + 4] * C1;
            iprevTi[i + 4].f1[2] = 0.0;
            iprevTi[i + 4].f1[6] = sa;
            iprevTi[i + 4].f1[10] = ca;
            iprevTi[i + 4].f1[14] = DH[i + 20];
            for (i0 = 0; i0 < 4; i0++) {
              iprevTi[i + 4].f1[3 + (i0 << 2)] = iv3[i0];
            }

            for (i0 = 0; i0 < 4; i0++) {
              for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
                   theta5_6_7_1_size_idx_1++) {
                b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] = 0.0;
                for (theta5_6_7_2_size_idx_0 = 0; theta5_6_7_2_size_idx_0 < 4;
                     theta5_6_7_2_size_idx_0++) {
                  b_baseTtool[i0 + (theta5_6_7_1_size_idx_1 << 2)] += baseTn[i +
                    3].f1[i0 + (theta5_6_7_2_size_idx_0 << 2)] * iprevTi[i + 4].
                    f1[theta5_6_7_2_size_idx_0 + (theta5_6_7_1_size_idx_1 << 2)];
                }
              }
            }

            for (i0 = 0; i0 < 4; i0++) {
              for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 4;
                   theta5_6_7_1_size_idx_1++) {
                baseTn[i + 4].f1[theta5_6_7_1_size_idx_1 + (i0 << 2)] =
                  b_baseTtool[theta5_6_7_1_size_idx_1 + (i0 << 2)];
              }
            }
          }

          //
          //  plotWAM(baseTn,2);
          //  title('Resultin pose');
          //   baseTn{8}
          for (i0 = 0; i0 < 3; i0++) {
            for (theta5_6_7_1_size_idx_1 = 0; theta5_6_7_1_size_idx_1 < 3;
                 theta5_6_7_1_size_idx_1++) {
              baseTtool[theta5_6_7_1_size_idx_1 + 3 * i0] = baseTn[7]
                .f1[theta5_6_7_1_size_idx_1 + (i0 << 2)];
            }
          }

          rotm2quat(baseTtool, b_p36_1);
          for (i0 = 0; i0 < 4; i0++) {
            temp[i0] = b_p36_1[i0];
          }

          for (i0 = 0; i0 < 3; i0++) {
            c_baseTn[i0] = baseTn[7].f1[12 + i0];
          }

          for (i0 = 0; i0 < 4; i0++) {
            c_baseTn[i0 + 3] = temp[i0];
          }

          for (i0 = 0; i0 < 7; i0++) {
            d_baseTn[i0] = c_baseTn[i0] - toolPoseDes[i0];
          }

          if (norm(d_baseTn) < 1.0E-6) {
            *success = 1.0;
          } else {
            //       warning('The result is not matched. Something wrong. ');
            //       PoseResult = PoseResult
            //       toolPoseDes = toolPoseDes
            //       plotWAM(baseTn,1);
            //       plotWAM(baseTnDes,1);
            //       plot3(p3(1),p3(2),p3(3),'co','LineWidth',5);
            //       plot3(p_best(1),p_best(2),p_best(3),'y*','LineWidth',5);
              for (int i=0;i<7;i++){
                  ROS_INFO("[IK solver WAM]: wrong solution:[%d]=%.6f",i,Theta_Jn[i]);
              }
            *success = 0.0;
          }
        }
      }
    }
  }
}

//
// File trailer for InverseKinematicsWAM.cpp
//
// [EOF]
//
