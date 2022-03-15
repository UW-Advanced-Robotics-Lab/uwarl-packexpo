//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 18-Sep-2019 00:20:57
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "KnobOperationWAM.h"
#include "main.h"
#include "KnobOperationWAM_terminate.h"
#include "KnobOperationWAM_initialize.h"

// Function Declarations
static void argInit_7x1_real_T(double result[7]);
static boolean_T argInit_boolean_T();
static double argInit_real_T();
static void main_KnobOperationWAM();

// Function Definitions

//
// Arguments    : double result[7]
// Return Type  : void
//
static void argInit_7x1_real_T(double result[7])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 7; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : boolean_T
//
static boolean_T argInit_boolean_T()
{
  return false;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_KnobOperationWAM()
{
  double dv3[7];
  double dv4[7];
  double Jp_cmd[287];
  double success;

  // Initialize function 'KnobOperationWAM' input arguments.
  // Initialize function input argument 'Jp_state'.
  // Initialize function input argument 'PoseKnobWrtWAM'.
  // Call the entry-point 'KnobOperationWAM'.
  argInit_7x1_real_T(dv3);
  argInit_7x1_real_T(dv4);
  KnobOperationWAM(dv3, argInit_boolean_T(), dv4, Jp_cmd, &success);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  KnobOperationWAM_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_KnobOperationWAM();

  // Terminate the application.
  // You do not need to do this more than one time.
  KnobOperationWAM_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
