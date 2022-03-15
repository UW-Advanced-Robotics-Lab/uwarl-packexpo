//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: InverseKinematicsWAM.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
//
#ifndef INVERSEKINEMATICSWAM_H
#define INVERSEKINEMATICSWAM_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "InverseKinematicsWAM_types.h"

// Function Declarations
extern void InverseKinematicsWAM(double toolPoseDes[7], const double JnCur[7],
  double Theta_Jn[7], double *success);

#endif

//
// File trailer for InverseKinematicsWAM.h
//
// [EOF]
//
