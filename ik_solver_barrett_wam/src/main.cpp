//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 07-Sep-2019 20:43:14
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
#include "InverseKinematicsWAM.h"
#include "main.h"
#include "InverseKinematicsWAM_terminate.h"
#include "InverseKinematicsWAM_initialize.h"

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

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/Empty.h>

#include "ik_solver_barrett_wam/InvKineWam.h"


// Function Declarations
static void argInit_7x1_real_T(double result[7]);
static double argInit_real_T();
//static bool main_InverseKinematicsWAM();
bool main_InverseKinematicsWAM(ik_solver_barrett_wam::InvKineWam::Request &req,ik_solver_barrett_wam::InvKineWam::Response &res);

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
bool main_InverseKinematicsWAM(ik_solver_barrett_wam::InvKineWam::Request &req,ik_solver_barrett_wam::InvKineWam::Response &res)
{
  double dvPoseToolDes[7];
  double dvJntCur[7];
  double JntSolved[7];
  double success;

  // Initialize function 'InverseKinematicsWAM' input arguments.
  // Initialize function input argument 'toolPoseDes'.
  // Initialize function input argument 'JnCur'.
  // Call the entry-point 'InverseKinematicsWAM'.
  argInit_7x1_real_T(dvPoseToolDes);
  argInit_7x1_real_T(dvJntCur);

  ROS_INFO(".");
  dvPoseToolDes[0] = req.poseToolDes.position.x;
  dvPoseToolDes[1] = req.poseToolDes.position.y;
  dvPoseToolDes[2] = req.poseToolDes.position.z;
  dvPoseToolDes[3] = req.poseToolDes.orientation.w;
  dvPoseToolDes[4] = req.poseToolDes.orientation.x;
  dvPoseToolDes[5] = req.poseToolDes.orientation.y;
  dvPoseToolDes[6] = req.poseToolDes.orientation.z;

//  dvJntCur[0] = req.jointsCur[0];
//  dvJntCur[1] = req.jointsCur[1];
//  dvJntCur[2] = req.jointsCur[2];
//  dvJntCur[3] = req.jointsCur[3];
//  dvJntCur[4] = req.jointsCur[4];
//  dvJntCur[5] = req.jointsCur[5];
//  dvJntCur[6] = req.jointsCur[6];

  dvJntCur[0] = 0.0;
  dvJntCur[1] = 0.0;
  dvJntCur[2] = 0.0;
  dvJntCur[3] = 0.0;
  dvJntCur[4] = 0.0;
  dvJntCur[5] = 0.0;
  dvJntCur[6] = 0.0;

  InverseKinematicsWAM(dvPoseToolDes, dvJntCur, JntSolved, &success);

  if (int(success) == 1){
      res.success = true;

      for (int i=0;i<7;i++){

          ROS_INFO("JntSolved[%d]=%4f",i,JntSolved[i]);
          res.jointsSolved[i] = JntSolved[i];
      }
      ROS_INFO("[IK solver WAM] IK has been Solved.");
  }
  else{
      res.success = false;

      for (int i=0;i<7;i++){
      res.jointsSolved[i] = req.jointsCur[i];
      }
      ROS_INFO("[IK solver WAM] Failed: Solving inv.kinematics failed.(out of range Jnt[%d])",int(success));
  }
  return true;
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int argc, char ** argv)
//int main(int argc, char **argv, const char * const [])
//int main(int argc, char **argv)
{

  //Initiate ROS
  ros::init(argc, argv, "ik_solver_wam");
  std::cout << "e";

//  ROS_INFO("Ready for the wam inverse kinematics.");

  ros::NodeHandle nh_;

  ros::ServiceServer srv_ik_wam;
  srv_ik_wam = nh_.advertiseService("ik_solver_wam/solve_inv_kine_wam", main_InverseKinematicsWAM);

  // Initialize the application.
  // You do not need to do this more than one time.
  InverseKinematicsWAM_initialize();

  ROS_INFO("Ready for the wam inverse kinematics.");

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.

  //    main_InverseKinematicsWAM();
  // Terminate the application.
  // You do not need to do this more than one time.
  ros::spin();

  InverseKinematicsWAM_terminate();

  return 0;
}


//
// File trailer for main.cpp
//
// [EOF]
//
