//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 17-Sep-2019 23:33:50
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

#include <string.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cfloat>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "knob_operation_wam/KnobOperation.h"
#include "knob_operation_wam/JointMove.h"

// Variable declaration
float jntcmd[7];

// Function Declarations
static void argInit_7x1_real_T(double result[7]);
static boolean_T argInit_boolean_T();
static double argInit_real_T();
//static void main_KnobOperationWAM();
bool main_KnobOperationWAM(knob_operation_wam::KnobOperation::Request &req,knob_operation_wam::KnobOperation::Response &res);

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
//static void main_KnobOperationWAM()
bool main_KnobOperationWAM(knob_operation_wam::KnobOperation::Request &req,knob_operation_wam::KnobOperation::Response &res)
{
  double CurJointWAM[7];
  double PoseKnobWrtWAM[7];
  double Jp_cmd[287];
  double success;
  bool isRevDir;
  int numCycle;

  knob_operation_wam::JointMove  srvJointMove;

  // Initialize function 'KnobOperationWAM' input arguments.
  // Initialize function input argument 'Jp_state'.
  // Initialize function input argument 'PoseKnobWrtWAM'.
  // Call the entry-point 'KnobOperationWAM'.
  argInit_7x1_real_T(CurJointWAM);
  argInit_7x1_real_T(PoseKnobWrtWAM);
  isRevDir = req.isRevDir;
  numCycle = req.cycleKnob;
  PoseKnobWrtWAM[0] = req.poseKnobWrtWAMbyCAM.position.x;
  PoseKnobWrtWAM[1] = req.poseKnobWrtWAMbyCAM.position.y;
  PoseKnobWrtWAM[2] = req.poseKnobWrtWAMbyCAM.position.z;
  PoseKnobWrtWAM[3] = req.poseKnobWrtWAMbyCAM.orientation.w;
  PoseKnobWrtWAM[4] = req.poseKnobWrtWAMbyCAM.orientation.x;
  PoseKnobWrtWAM[5] = req.poseKnobWrtWAMbyCAM.orientation.y;
  PoseKnobWrtWAM[6] = req.poseKnobWrtWAMbyCAM.orientation.z;

  for (int j=0;j<7;j++){
     CurJointWAM[j] = req.jointsCur[j];
  }

//  success = KnobOperationWAM(CurJointWAM, isRevDir, PoseKnobWrtWAM);
  KnobOperationWAM(CurJointWAM, isRevDir, PoseKnobWrtWAM, Jp_cmd, &success);
  int j;
  if (success == 1){
      res.success = true;
      for (int i=0;i<30;i++){
          j= i*7;
          ROS_INFO("JntCmd[%d]:(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",i,Jp_cmd[j],Jp_cmd[j+1],Jp_cmd[j+2],Jp_cmd[j+3],Jp_cmd[j+4],Jp_cmd[j+5],Jp_cmd[j+6]);
          for (int k=0;k<7;k++){
              jntcmd[k]=Jp_cmd[j+k];
          }
          srvJointMove.request.joints = {jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]};
          srv_joint_move.call(srvJointMove);
          ros::Duration(0.1).sleep();
      }

  }
  else{
      res.success = false;
      ROS_INFO("Knob Operation Failed: Error code=%.1f",success);
  }

  return true;
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
//int main(int, const char * const [])
int main(int argc, char ** argv)
{
    //Initiate ROS
    ros::init(argc, argv, "knob_operation_wam");
    std::cout << "e";

    ros::NodeHandle nh_;

    ros::ServiceServer srv_knob_operation_wam;
    srv_knob_operation_wam = nh_.advertiseService("knob_operation_wam/send_jntcmd_knob_operation", main_KnobOperationWAM);

    ros::ServiceClient srv_joint_move;
    srv_joint_move = nh_.serviceClient<knob_operation_wam::JointMove>("wam/joint_move");

  // Initialize the application.
  // You do not need to do this more than one time.
  KnobOperationWAM_initialize();

  ROS_INFO("Ready for the knob operation.");

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
//  main_KnobOperationWAM();
  ros::spin();

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
