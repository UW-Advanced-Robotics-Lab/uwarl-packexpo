//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: KnobOperationWAM.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//
#ifndef KNOBOPERATIONWAM_H
#define KNOBOPERATIONWAM_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "KnobOperationWAM_types.h"

// Function Declarations
extern void KnobOperationWAM(const double Jp_state[7], boolean_T isRevDir,
  double PoseKnobWrtWAM[7], double Jp_cmd[287], double *success);

#endif

//
// File trailer for KnobOperationWAM.h
//
// [EOF]
//
