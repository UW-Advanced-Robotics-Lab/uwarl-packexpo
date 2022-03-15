//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: KnobOperationWAM.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "KnobOperationWAM.h"
#include "mrdivide.h"
#include "cross.h"
#include "FwdKineWAM.h"
#include "TransformTrajWrtWAM.h"

// Function Definitions

//
// Initialize Parameters
//  For test, obtain the initial values
//  Receive the joint data
//  Jp_state = [0,-1,1,2.5,-1,0.07,1]';
// Arguments    : const double Jp_state[7]
//                boolean_T isRevDir
//                double PoseKnobWrtWAM[7]
//                double Jp_cmd[287]
//                double *success
// Return Type  : void
//
void KnobOperationWAM(const double Jp_state[7], boolean_T isRevDir, double
                      PoseKnobWrtWAM[7], double Jp_cmd[287], double *success)
{
  double Pn_init[24];
  cell_wrap_0 baseTn_init[8];
  cell_wrap_0 iprevTi_init[8];
  cell_wrap_0 iTprevi_init[8];
  int i0;
  static const double dv0[123] = { 0.0, 0.001231165940486223,
    0.0048943483704846358, 0.010899347581163205, 0.019098300562505249,
    0.02928932188134524, 0.041221474770752678, 0.054600950026045314,
    0.069098300562505252, 0.0843565534959769, 0.099999999999999992,
    0.11564344650402307, 0.13090169943749475, 0.14539904997395467,
    0.15877852522924729, 0.17071067811865476, 0.18090169943749473,
    0.18910065241883678, 0.19510565162951538, 0.19876883405951379, 0.2,
    0.19876883405951379, 0.19510565162951538, 0.18910065241883681,
    0.18090169943749476, 0.17071067811865476, 0.15877852522924735,
    0.1453990499739547, 0.13090169943749477, 0.11564344650402313,
    0.10000000000000003, 0.084356553495977038, 0.0690983005625053,
    0.054600950026045272, 0.041221474770752713, 0.029289321881345337,
    0.019098300562505277, 0.010899347581163191, 0.0048943483704846635,
    0.0012311659404862507, 0.0, 1.2246467991473533E-17, -0.015643446504023075,
    -0.030901699437494729, -0.045399049973954671, -0.058778525229247307,
    -0.070710678118654752, -0.080901699437494742, -0.089100652418836787,
    -0.095105651629515356, -0.098768834059513769, -0.1, -0.098768834059513783,
    -0.09510565162951537, -0.0891006524188368, -0.080901699437494756,
    -0.070710678118654766, -0.058778525229247341, -0.0453990499739547,
    -0.030901699437494764, -0.015643446504023113, -2.4492935982947065E-17,
    0.015643446504023061, 0.030901699437494719, 0.045399049973954664,
    0.058778525229247293, 0.070710678118654738, 0.080901699437494728,
    0.089100652418836787, 0.095105651629515356, 0.098768834059513769, 0.1,
    0.0987688340595138, 0.09510565162951537, 0.089100652418836759,
    0.080901699437494756, 0.070710678118654835, 0.058778525229247341,
    0.045399049973954636, 0.030901699437494781, 0.01564344650402321,
    3.6739403974420595E-17, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0 };

  double CartTrajWristWrtWAM[123];
  int b_success;
  int i;
  double Jp_state_new[7];
  double delP[6];
  int iii;
  boolean_T exitg2;
  int ii;
  double J[42];
  static const double b[3] = { 0.0, 0.0, 1.0 };

  double b_baseTn_init[3];
  double c_baseTn_init[3];
  int i1;
  double dv1[42];
  double b_J[36];
  double d0;
  int exitg1;
  static const double theta_min[7] = { -2.6, -2.0, -2.8, -0.9, -4.76, -1.6, -3.0
  };

  static const double theta_max[7] = { 2.6, 2.0, 2.8, 3.1, 1.24, 1.6, 3.0 };

  //  This is for the knob operation of WAM that has circular path
  //  Jeong-woo Han, jeong-woo.han@uwaterloo.ca, 20190915
  //  Create the circular trajectory, wrt Knob frame
  FwdKineWAM(Jp_state, Pn_init, baseTn_init, iprevTi_init, iTprevi_init);

  //  Jp_state: 7x1 joint angles, rad
  //  plotWAM(baseTn_init,1);
  //  PoseKnobWrtWAM = zeros(7,1);
  for (i0 = 0; i0 < 3; i0++) {
    PoseKnobWrtWAM[i0] = baseTn_init[6].f1[12 + i0];
  }

  //  PoseKnobWrtWAM(4:7) = rotm2quat(baseTn_init{7}(1:3,1:3))';
  //  Below requires computation of orientation of knob
  //  Rz*Rx
  //  PoseKnobWrtWAM(4:7) = [0.9848;0;-0.1736;0;];
  //  Transform trajectory
  TransformTrajWrtWAM(PoseKnobWrtWAM, dv0, CartTrajWristWrtWAM);

  //  isRevDir = 1; % 0(false): normal direction, 1(true): reversed direction
  b_success = 1;
  memset(&Jp_cmd[0], 0, 287U * sizeof(double));
  for (i = 0; i < 7; i++) {
    Jp_state_new[i] = Jp_state[i];
  }

  //  Jp_state: 7x1 joint angles, rad
  for (i = 0; i < 6; i++) {
    delP[i] = 0.0;
  }

  //  Solve the inv. kinematics using Jacobian
  iii = 0;
  exitg2 = false;
  while ((!exitg2) && (iii < 41)) {
    if (!isRevDir) {
      //  normal direction
      ii = iii;
    } else {
      //  opposite direction
      ii = 40 - iii;
    }

    //  Solve current pose with given angles
    FwdKineWAM(Jp_state_new, Pn_init, baseTn_init, iprevTi_init, iTprevi_init);

    //  Jp_state: 7x1 joint angles, rad
    //  Pold = Pnew;
    for (i = 0; i < 3; i++) {
      delP[i] = CartTrajWristWrtWAM[ii + 41 * i] - Pn_init[18 + i];
      delP[i + 3] = 0.0;
    }

    //  Jacobian Calculation from current
    memset(&J[0], 0, 42U * sizeof(double));
    cross(b, *(double (*)[3])&baseTn_init[6].f1[12], *(double (*)[3])&J[0]);
    for (i0 = 0; i0 < 3; i0++) {
      J[3 + i0] = b[i0];
    }

    for (i = 0; i < 6; i++) {
      for (i0 = 0; i0 < 3; i0++) {
        b_baseTn_init[i0] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          b_baseTn_init[i0] += baseTn_init[i].f1[i0 + (i1 << 2)] * b[i1];
        }

        c_baseTn_init[i0] = baseTn_init[6].f1[12 + i0] - baseTn_init[i].f1[12 +
          i0];
      }

      cross(b_baseTn_init, c_baseTn_init, *(double (*)[3])&J[6 * (i + 1)]);
      for (i0 = 0; i0 < 3; i0++) {
        J[(i0 + 6 * (i + 1)) + 3] = 0.0;
        for (i1 = 0; i1 < 3; i1++) {
          J[(i0 + 6 * (i + 1)) + 3] += baseTn_init[i].f1[i0 + (i1 << 2)] * b[i1];
        }
      }
    }

    //  Inv of Jacobian, minimum energy
    //  compute inverse
    //  update
    for (i0 = 0; i0 < 6; i0++) {
      for (i1 = 0; i1 < 7; i1++) {
        dv1[i1 + 7 * i0] = J[i0 + 6 * i1];
      }

      for (i1 = 0; i1 < 6; i1++) {
        b_J[i0 + 6 * i1] = 0.0;
        for (ii = 0; ii < 7; ii++) {
          b_J[i0 + 6 * i1] += J[i0 + 6 * ii] * J[i1 + 6 * ii];
        }
      }
    }

    mrdivide(dv1, b_J);
    for (i0 = 0; i0 < 7; i0++) {
      d0 = 0.0;
      for (i1 = 0; i1 < 6; i1++) {
        d0 += dv1[i0 + 7 * i1] * delP[i1];
      }

      Jp_state_new[i0] += d0;
    }

    //  Check Min/Max of each angle
    ii = 0;
    do {
      exitg1 = 0;
      if (ii < 7) {
        if ((Jp_state_new[ii] < theta_min[ii]) || (Jp_state_new[ii] >
             theta_max[ii])) {
          b_success = -ii - 1;
          exitg1 = 1;
        } else {
          ii++;
        }
      } else {
        for (i0 = 0; i0 < 7; i0++) {
          Jp_cmd[i0 + 7 * iii] = Jp_state_new[i0];
        }

        iii++;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  *success = b_success;

  //  wamPknob = PoseKnobWrtWAM(1:3);
  //
  //  figure(1); clf;
  //
  //  plot3(CartRefTrajKnob(:,1),CartRefTrajKnob(:,2),CartRefTrajKnob(:,3),'b-*'); 
  //  hold on;
  //  plot3(CartTrajWristWrtWAM(:,1),CartTrajWristWrtWAM(:,2),CartTrajWristWrtWAM(:,3),'r-o'); 
  //  axis equal;
  //  grid on;
  //  xlabel('x'); ylabel('y'); zlabel('z');
  //  for i=1:size(CartTrajWristWrtWAM,1)
  //     theta = Jp_cmd(:,i);
  //     [Pn, baseTn, iprevTi, iTprevi] = FwdKineWAM(theta); % Jp_state: 7x1 joint angles, rad 
  //     if mod(i,8) == 1
  //      hFig1 = plotWAM(baseTn,1);
  //     end
  //      plot3(Pn(1,8),Pn(2,8),Pn(3,8),'c*');
  //      drawnow;
  //  %     pause(0.1);
  //  end
  //  function [Jp_cmd, success] = InvKineJacobian(CartTrajWristWrtWAM, Jp_state,isRevDir) 
  //  theta_min = [-2.6; -2; -2.8; -0.9; -4.76; -1.6; -3;];
  //  theta_max = [2.6; 2; 2.8; 3.1; 1.24; 1.6; 3;];
  //  success = 1;
  //
  //  NumPtOnTraj = size(CartTrajWristWrtWAM,1);
  //  Jp_cmd = zeros(7,NumPtOnTraj);
  //
  //  Jp_state_old = Jp_state;
  //  Jp_state_new = Jp_state;
  //
  //  [Pn, baseTn, iprevTi, iTprevi] = FwdKineWAM(Jp_state_new); % Jp_state: 7x1 joint angles, rad 
  //
  //  Pnew = Pn(:,7);
  //  delP = zeros(6,1);
  //
  //  % Solve the inv. kinematics using Jacobian
  //  for iii = 1:NumPtOnTraj
  //  if isRevDir == 0 % normal direction
  //      ii = iii;
  //  else % opposite direction
  //      ii = NumPtOnTraj - iii +1;
  //  end
  //  Jp_state_old = Jp_state_new;
  //  % Solve current pose with given angles
  //  [Pn, baseTn, iprevTi, iTprevi] = FwdKineWAM(Jp_state_old); % Jp_state: 7x1 joint angles, rad 
  //  % Pold = Pnew;
  //  Pold = Pn(:,7);
  //  Pnew = CartTrajWristWrtWAM(ii,:)';
  //  delP(1:3) = Pnew - Pold;
  //  delP(4:6) = [0,0,0]';
  //
  //  % Jacobian Calculation from current
  //  J = zeros(6,7);
  //  J(1:3,1) = cross(eye(3)*[0 0 1]',baseTn{7}(1:3,4));
  //  J(4:6,1) = [0 0 1];
  //  for i=2:7
  //     J(1:3,i) = cross(baseTn{i-1}(1:3,1:3)*[0 0 1]',baseTn{7}(1:3,4)-baseTn{i-1}(1:3,4)); 
  //     J(4:6,i) = baseTn{i-1}(1:3,1:3)*[0 0 1]';
  //  end
  //  % Inv of Jacobian, minimum energy
  //  psJ = J'/(J*J');
  //  % compute inverse
  //  delQ = psJ*delP;
  //  % update
  //  Jp_state_new = Jp_state_old + delQ;
  //  % Check Min/Max of each angle
  //  for jj=1:7
  //      if ((Jp_state_new(jj) < theta_min(jj))||(Jp_state_new(jj) > theta_max(jj))) 
  //          success = -jj;
  //          return;
  //      end
  //  end
  //  Jp_cmd(:,iii) = Jp_state_new;
  //  end
  //  function CartTrajWristWrtWAM = TransformTrajWrtWAM(PoseKnobWrtWAM, CartTrajWrtKnob) 
  //
  //  AngleOfKnob = 20;
  //
  //  wamPositionWrist = PoseKnobWrtWAM(1:3);
  //  temp = quat2rotm(PoseKnobWrtWAM(4:7)');
  //  n_knob = temp(:,3);
  //
  //  x = CartTrajWrtKnob(:,1)';
  //  y = CartTrajWrtKnob(:,2)';
  //  z = CartTrajWrtKnob(:,3)';
  //
  //  wamTwrist = eye(4);
  //
  //  phi = pi-atan2(n_knob(2),n_knob(1));
  //  psi = -AngleOfKnob*pi/180; % angle of knob
  //  wamRwrist = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1;]*...
  //             [cos(psi) 0 sin(psi); 0 1 0; -sin(psi) 0 cos(psi);];
  //  wamTwrist(1:3,1:3) = wamRwrist;
  //  wamTwrist(1:3,4) = wamPositionWrist;
  //
  //  knobPcircle = [x;y;z;ones(size(x))];
  //  wamPcircle = wamTwrist*knobPcircle;
  //
  //  CartTrajWristWrtWAM = wamPcircle(1:3,:)';
  //  function CartTrajWrtKnob=createRefCartTraj()
  //      NumPt = 40;
  //      r = 0.1;
  //      th = 0:(2*pi/NumPt):2*pi';
  //
  //      x = r*cos(th+pi)+r;
  //      y = r*sin(th+pi);
  //      z = 0*th;
  //
  //      CartTrajWrtKnob = [x;y;z;]'; % Positions of trajectoryes
}

//
// File trailer for KnobOperationWAM.cpp
//
// [EOF]
//
