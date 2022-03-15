#include <string.h>
#include <sstream>
#include <iostream>
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
#include "sensor_msgs/JointState.h"
#include "ik_solver_barrett_wam/InvKineWam.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// For WAM operation (Related services)
#include "uwarl_tasks/PoseMove.h"
#include "uwarl_tasks/CartPosMove.h"
#include "uwarl_tasks/OrtnMove.h"
#include "uwarl_tasks/JointMove.h"
#include "uwarl_tasks/BHandFingerVel.h"
#include "uwarl_tasks/BHandGraspVel.h"
#include "uwarl_tasks/BHandGraspPos.h"
#include "uwarl_tasks/BHandSpreadPos.h"
#include "uwarl_tasks/BHandSpreadVel.h"
#include "uwarl_tasks/Hold.h"
#include "uwarl_tasks/GravityComp.h"
#include <std_srvs/Empty.h>

#include <math.h>

#define ROBOT_NUMBER_SUMMIT 0
#define ROBOT_NUMBER_WAM 1

#define WAM_DOF 7

// Targets for Navigation
#define TABLE_A 0
#define TABLE_B 1
#define KNOB 2

// mode
#define WAIT 0
#define MOVE_TO_TABLE_A_FROM_ANYWHERE 1
#define PICK_BOTTLE_FROM_A 2
#define MOVE_TO_TABLE_B_FROM_A 3
#define PLACE_BOTTLE_ON_B 4
#define MOVE_TO_KNOB 5
#define TIGHTEN_KNOB 6
#define RELEASE_KNOB 7
#define MOVE_TO_TABLE_B_FROM_KNOB 8
#define PICK_BOTTLE_FROM_B 9
#define MOVE_TO_TABLE_A_FROM_B 10
#define PLACE_BOTTLE_ON_A 11

#define NUM_OF_MODES 11

#define COUNT_FINE_NAV 30

const float TRANS_WAM_TO_ROBOTMARKER_X = -0.330; // m
const float TRANS_WAM_TO_ROBOTMARKER_Y = -0.145; // m
//const float TRANS_WAM_TO_ROBOTMARKER_Z = -0.186; // m
const float TRANS_WAM_TO_ROBOTMARKER_Z = -0.195; // m
const float ROTATION_WAM_TO_ROBOTMARKER_ROLL = 1.57079632679; // rad
const float ROTATION_WAM_TO_ROBOTMARKER_PITCH = 0.0; // rad
const float ROTATION_WAM_TO_ROBOTMARKER_YAW = 0.0; // rad

const float BHAND_GRASP_ORNT_ROLL = 1.57079632679;
const float BHAND_GRASP_ORNT_PITCH = 0.0;
const float BHAND_GRASP_ORNT_YAW = 1.57079632679;

// Location of the spot on the rail to place object, wrt rail marker coordinate
const float TRANS_RAIL_MARKER_TO_PLACE_X = 0.0; // m
const float TRANS_RAIL_MARKER_TO_PLACE_Y = -0.60; // m
const float TRANS_RAIL_MARKER_TO_PLACE_Z = 0.06; // m

//static const float ForNav = {0,-1.6,0,2.7,0,0.47,1.57};
//static const float EndPt = {0,-1.2,0,2.3,0,0.47,1.57};
//static const float StartPt = {0,-0.4,0,1.2,0,0,1.57};
//static const float Placing = {0,-0.4,0,1.5,0,0.47,1.57};

class ServiceCore
{
public:
  ServiceCore()
  {
    fnInitParam(); // load parameters and init
    fnSystemInit(); // system handles init

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
      fnCheckModeChange();         // Check if mode should be changed to the next
      fnDoActionWhenModeChange();  // Do action if mode is changed
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  void fnInitParam()
  {
     ROS_INFO("Init. Param. started");

    // SUMMIT: Get the target poses for navigation: Table A, Table B, knob
    nh_.getParam("tableA_pose_summit/position", target_pose_position);
    nh_.getParam("tableA_pose_summit/orientation", target_pose_orientation);

    poseStampedTargetNav[0].header.frame_id = "summit_xl_map";
    poseStampedTargetNav[0].header.stamp = ros::Time::now();
 
    poseStampedTargetNav[0].pose.position.x = target_pose_position[0];
    poseStampedTargetNav[0].pose.position.y = target_pose_position[1];
    poseStampedTargetNav[0].pose.position.z = target_pose_position[2];

    poseStampedTargetNav[0].pose.orientation.x = target_pose_orientation[0];
    poseStampedTargetNav[0].pose.orientation.y = target_pose_orientation[1];
    poseStampedTargetNav[0].pose.orientation.z = target_pose_orientation[2];
    poseStampedTargetNav[0].pose.orientation.w = target_pose_orientation[3];

    nh_.getParam("tableB_pose_summit/position", target_pose_position);
    nh_.getParam("tableB_pose_summit/orientation", target_pose_orientation);

    poseStampedTargetNav[1].header.frame_id = "summit_xl_map";
    poseStampedTargetNav[1].header.stamp = ros::Time::now();

    poseStampedTargetNav[1].pose.position.x = target_pose_position[0];
    poseStampedTargetNav[1].pose.position.y = target_pose_position[1];
    poseStampedTargetNav[1].pose.position.z = target_pose_position[2];

    poseStampedTargetNav[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedTargetNav[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedTargetNav[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedTargetNav[1].pose.orientation.w = target_pose_orientation[3];

    nh_.getParam("knob_pose_summit/position", target_pose_position);
    nh_.getParam("knob_pose_summit/orientation", target_pose_orientation);

    poseStampedTargetNav[2].header.frame_id = "summit_xl_map";
    poseStampedTargetNav[2].header.stamp = ros::Time::now();

    poseStampedTargetNav[2].pose.position.x = target_pose_position[0];
    poseStampedTargetNav[2].pose.position.y = target_pose_position[1];
    poseStampedTargetNav[2].pose.position.z = target_pose_position[2];

    poseStampedTargetNav[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedTargetNav[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedTargetNav[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedTargetNav[2].pose.orientation.w = target_pose_orientation[3];

//    /////////////////////////////////////// Below code is just for the case of fine adjustment using cameras
    /*
//    // WAM
//    nh_.getParam("bottleA_pose_WAM/position", target_pose_position);
//    nh_.getParam("bottleA_pose_WAM/orientation", target_pose_orientation);

//    poseStampedTargetWAM[0].header.frame_id = "summit_xl_map";
//    poseStampedTargetWAM[0].header.stamp = ros::Time::now();

//    poseStampedTargetWAM[0].pose.position.x = target_pose_position[0];
//    poseStampedTargetWAM[0].pose.position.y = target_pose_position[1];
//    poseStampedTargetWAM[0].pose.position.z = target_pose_position[2];

//    poseStampedTargetWAM[0].pose.orientation.x = target_pose_orientation[0];
//    poseStampedTargetWAM[0].pose.orientation.y = target_pose_orientation[1];
//    poseStampedTargetWAM[0].pose.orientation.z = target_pose_orientation[2];
//    poseStampedTargetWAM[0].pose.orientation.w = target_pose_orientation[3];

//    nh_.getParam("bottleB_pose_WAM/position", target_pose_position);
//    nh_.getParam("bottleB_pose_WAM/orientation", target_pose_orientation);

//    poseStampedTargetWAM[1].header.frame_id = "summit_xl_map";
//    poseStampedTargetWAM[1].header.stamp = ros::Time::now();

//    poseStampedTargetWAM[1].pose.position.x = target_pose_position[0];
//    poseStampedTargetWAM[1].pose.position.y = target_pose_position[1];
//    poseStampedTargetWAM[1].pose.position.z = target_pose_position[2];

//    poseStampedTargetWAM[1].pose.orientation.x = target_pose_orientation[0];
//    poseStampedTargetWAM[1].pose.orientation.y = target_pose_orientation[1];
//    poseStampedTargetWAM[1].pose.orientation.z = target_pose_orientation[2];
//    poseStampedTargetWAM[1].pose.orientation.w = target_pose_orientation[3];

//    nh_.getParam("desired_cam_pose_summit_Table_A/position", target_pose_position);
//    nh_.getParam("desired_cam_pose_summit_Table_A/orientation", target_pose_orientation);

//    poseTargetNavByCam[0].header.frame_id = "zed_left_camera_frame";
//    poseStampedTargetNavByCam[1].header.stamp = ros::Time::now();

//    poseTargetNavByCam[0].position.x = target_pose_position[0];
//    poseTargetNavByCam[0].position.y = target_pose_position[1];
//    poseTargetNavByCam[0].position.z = target_pose_position[2];

//    poseTargetNavByCam[0].orientation.x = target_pose_orientation[0];
//    poseTargetNavByCam[0].orientation.y = target_pose_orientation[1];
//    poseTargetNavByCam[0].orientation.z = target_pose_orientation[2];
//    poseTargetNavByCam[0].orientation.w = target_pose_orientation[3];

//    nh_.getParam("desired_cam_pose_summit_Table_B/position", target_pose_position);
//    nh_.getParam("desired_cam_pose_summit_Table_B/orientation", target_pose_orientation);

//    poseTargetNavByCam[1].header.frame_id = "zed_left_camera_frame";
//    poseStampedTargetNavByCam[1].header.stamp = ros::Time::now();

//    poseTargetNavByCam[1].position.x = target_pose_position[0];
//    poseTargetNavByCam[1].position.y = target_pose_position[1];
//    poseTargetNavByCam[1].position.z = target_pose_position[2];

//    poseTargetNavByCam[1].orientation.x = target_pose_orientation[0];
//    poseTargetNavByCam[1].orientation.y = target_pose_orientation[1];
//    poseTargetNavByCam[1].orientation.z = target_pose_orientation[2];
//    poseTargetNavByCam[1].orientation.w = target_pose_orientation[3];


//    nh_.getParam("desired_cam_pose_summit_Knob/position", target_pose_position);
//    nh_.getParam("desired_cam_pose_summit_Knob/orientation", target_pose_orientation);

////    poseTargetNavByCam[2].header.frame_id = "zed_left_camera_frame";
////    poseStampedTargetNavByCam[1].header.stamp = ros::Time::now();

//    poseTargetNavByCam[2].position.x = target_pose_position[0];
//    poseTargetNavByCam[2].position.y = target_pose_position[1];
//    poseTargetNavByCam[2].position.z = target_pose_position[2];

//    poseTargetNavByCam[2].orientation.x = target_pose_orientation[0];
//    poseTargetNavByCam[2].orientation.y = target_pose_orientation[1];
//    poseTargetNavByCam[2].orientation.z = target_pose_orientation[2];
//    poseTargetNavByCam[2].orientation.w = target_pose_orientation[3];

//    poseTargetNavByCamThisTime = poseTargetNavByCam[0];

//    poseFineAdjustInAmclFrame.header.frame_id = "summit_xl_map";
//    poseFineAdjustInAmclFrame.header.stamp = ros::Time::now();
*/

    poseStampedObj1ByCam1.header.frame_id = "";
    poseStampedObj1ByCam1.header.stamp = ros::Time::now();

    poseStampedObj1ByCam1.pose.position.x = 0;
    poseStampedObj1ByCam1.pose.position.y = 0;
    poseStampedObj1ByCam1.pose.position.z = 0;

    poseStampedObj1ByCam1.pose.orientation.x = 0;
    poseStampedObj1ByCam1.pose.orientation.y = 0;
    poseStampedObj1ByCam1.pose.orientation.z = 0;
    poseStampedObj1ByCam1.pose.orientation.w = 1;

    poseStampedObj2ByCam1 = poseStampedObj1ByCam1;
    poseStampedObj3ByCam1 = poseStampedObj1ByCam1;
    poseStampedObj1ByCam2 = poseStampedObj1ByCam1;
    poseStampedObj2ByCam2 = poseStampedObj1ByCam1;
    poseStampedObj3ByCam2 = poseStampedObj1ByCam1;
//    poseArrayKnobByCam2.poses[0] = poseStampedObj1ByCam1.pose;

    // Static places (target location of placing)
    nh_.getParam("place_Table_A_obj1_wrt_cam1/position", target_pose_position);
    nh_.getParam("place_Table_A_obj1_wrt_cam1/orientation", target_pose_orientation);
    poseObj1OnTableACam1.position.x = target_pose_position[0];
    poseObj1OnTableACam1.position.y = target_pose_position[1];
    poseObj1OnTableACam1.position.z = target_pose_position[2];
    poseObj1OnTableACam1.orientation.x = target_pose_orientation[0]; // orientation is not used. for convenience of code
    poseObj1OnTableACam1.orientation.y = target_pose_orientation[1];
    poseObj1OnTableACam1.orientation.z = target_pose_orientation[2];
    poseObj1OnTableACam1.orientation.w = target_pose_orientation[3];

    nh_.getParam("place_Table_A_obj2_wrt_cam1/position", target_pose_position);
    nh_.getParam("place_Table_A_obj2_wrt_cam1/orientation", target_pose_orientation);
    poseObj2OnTableACam1.position.x = target_pose_position[0];
    poseObj2OnTableACam1.position.y = target_pose_position[1];
    poseObj2OnTableACam1.position.z = target_pose_position[2];
    poseObj2OnTableACam1.orientation.x = target_pose_orientation[0]; // orientation is not used. for convenience of code
    poseObj2OnTableACam1.orientation.y = target_pose_orientation[1];
    poseObj2OnTableACam1.orientation.z = target_pose_orientation[2];
    poseObj2OnTableACam1.orientation.w = target_pose_orientation[3];

    nh_.getParam("place_Table_A_obj3_wrt_cam1/position", target_pose_position);
    nh_.getParam("place_Table_A_obj3_wrt_cam1/orientation", target_pose_orientation);
    poseObj3OnTableACam1.position.x = target_pose_position[0];
    poseObj3OnTableACam1.position.y = target_pose_position[1];
    poseObj3OnTableACam1.position.z = target_pose_position[2];
    poseObj3OnTableACam1.orientation.x = target_pose_orientation[0]; // orientation is not used. for convenience of code
    poseObj3OnTableACam1.orientation.y = target_pose_orientation[1];
    poseObj3OnTableACam1.orientation.z = target_pose_orientation[2];
    poseObj3OnTableACam1.orientation.w = target_pose_orientation[3];

    nh_.getParam("place_Rail_obj_wrt_cam2/position", target_pose_position);
    nh_.getParam("place_Rail_obj_wrt_cam2/orientation", target_pose_orientation);
    poseRailByCam2.position.x = target_pose_position[0];
    poseRailByCam2.position.y = target_pose_position[1];
    poseRailByCam2.position.z = target_pose_position[2];
    poseRailByCam2.orientation.x = target_pose_orientation[0]; // orientation is not used. for convenience of code
    poseRailByCam2.orientation.y = target_pose_orientation[1];
    poseRailByCam2.orientation.z = target_pose_orientation[2];
    poseRailByCam2.orientation.w = target_pose_orientation[3];

    nh_.getParam("pose_Knob_wrt_cam2/position", target_pose_position);
    nh_.getParam("pose_Knob_wrt_cam2/orientation", target_pose_orientation);
    poseKnobByCam2.position.x = target_pose_position[0];
    poseKnobByCam2.position.y = target_pose_position[1];
    poseKnobByCam2.position.z = target_pose_position[2];
    poseKnobByCam2.orientation.x = target_pose_orientation[0];
    poseKnobByCam2.orientation.y = target_pose_orientation[1];
    poseKnobByCam2.orientation.z = target_pose_orientation[2];
    poseKnobByCam2.orientation.w = target_pose_orientation[3];

    for (int i=0;i<7;i++)
    {
        jntcmd[i] =0.0;
    }
    // Barrett hand desired orientation for grasping (for now, it is static. TBD depending on the orientation of objects), note that RPY is performed wrt fixed axis (i.e. base of the WAM)
    Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,BHAND_GRASP_ORNT_YAW);
    quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
    ROS_INFO("quatBHandGraspObjOrtn={%.4f,%.4f,%.4f,%.4f}",quatBHandGraspObjOrtn.x,quatBHandGraspObjOrtn.y,quatBHandGraspObjOrtn.z,quatBHandGraspObjOrtn.w);
    tfWamToRobotMarker.setOrigin(tf2::Vector3(TRANS_WAM_TO_ROBOTMARKER_X,TRANS_WAM_TO_ROBOTMARKER_Y,TRANS_WAM_TO_ROBOTMARKER_Z));
    // tfWamToRobotMarker.setRotation(tf::Quaternion(70710678118655,0.0,0.0,70710678118655));
    tf2::Quaternion q;
    q.setRPY(ROTATION_WAM_TO_ROBOTMARKER_ROLL,ROTATION_WAM_TO_ROBOTMARKER_PITCH, ROTATION_WAM_TO_ROBOTMARKER_YAW);
    tfWamToRobotMarker.setRotation(q);

    yaw_for_grasp = BHAND_GRASP_ORNT_YAW;

    ROS_INFO("Init. Param has been done");
  }

  void fnSystemInit()
  {
      ROS_INFO("Init. System. started");

      pubServiceStatusSummit = nh_.advertise<std_msgs::String>("/summit_xl/service_status", 1);
      // pubServiceStatusWAM = nh_.advertise<std_msgs::String>("/wam/service_status", 1);

      pubPoseStampedSummit = nh_.advertise<geometry_msgs::PoseStamped>("/summit_xl/move_base_simple/goal", 1);
      // pubPoseStampedWAM = nh_.advertise<geometry_msgs::PoseStamped>("/wam/move_base_simple/goal", 1);

      pubPoseObj1FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj1", 1);
      pubPoseObj2FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj2", 1);
      pubPoseObj3FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj3", 1);
      pubPoseRailFromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/rail", 1);
      pubPoseKnobFromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/knob", 1);

      sub_enable_auto_operation = nh_.subscribe("/enable_auto_operation", 1, &ServiceCore::cbReceiveEnableAutoOperation, this);

      sub_arrival_status_summit = nh_.subscribe("/summit_xl/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusSUMMIT, this);
      // sub_arrival_status_wam = nh_.subscribe("/tb3g/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusWAM, this);

  //    sub_robot_marker_pose_by_cam1 = nh_.subscribe("/aruco_single/pose", 1, &ServiceCore::cbReceivePoseRobotMarkerFromCam1, this);
      sub_robot_marker_pose_by_cam1 = nh_.subscribe("/zed1/aruco_single_robot/pose", 1, &ServiceCore::cbReceivePoseRobotMarkerFromCam1, this);
      sub_robot_marker_pose_by_cam2 = nh_.subscribe("/zed2/aruco_single_robot/pose", 1, &ServiceCore::cbReceivePoseRobotMarkerFromCam2, this);

  //    sub_obj1_pose_by_cam1 = nh_.subscribe("/dope/pose_soup", 1, &ServiceCore::cbReceivePoseObj1FromCam1, this);
      sub_obj1_pose_by_cam1 = nh_.subscribe("/zed1/aruco_single_obj1/pose", 1, &ServiceCore::cbReceivePoseObj1FromCam1, this);
      sub_obj1_pose_by_cam2 = nh_.subscribe("/zed2/aruco_single_obj1/pose", 1, &ServiceCore::cbReceivePoseObj1FromCam2, this);
      sub_obj2_pose_by_cam1 = nh_.subscribe("/zed1/aruco_single_obj2/pose", 1, &ServiceCore::cbReceivePoseObj2FromCam1, this);
      sub_obj2_pose_by_cam2 = nh_.subscribe("/zed2/aruco_single_obj2/pose", 1, &ServiceCore::cbReceivePoseObj2FromCam2, this);
      sub_obj3_pose_by_cam1 = nh_.subscribe("/zed1/aruco_single_obj3/pose", 1, &ServiceCore::cbReceivePoseObj3FromCam1, this);
      sub_obj3_pose_by_cam2 = nh_.subscribe("/zed2/aruco_single_obj3/pose", 1, &ServiceCore::cbReceivePoseObj3FromCam2, this);
      sub_knob_pose_by_cam2 = nh_.subscribe("zed2/whycon/poses", 1, &ServiceCore::cbReceivePoseKnobFromCam2, this);
      //      sub_rail_pose_by_cam2 = nh_.subscribe("/zed2/aruco_single_obj3/pose", 1, &ServiceCore::cbReceivePoseRailFromCam2, this);
      // sub_arrival_status_wam = nh_.subscribe("/tb3g/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusWAM, this);

      sub_move_base_simple_goal = nh_.subscribe("/summit_xl/move_base_simple/goal", 1, &ServiceCore::cbReceiveMoveBaseSimpleGoal, this);
      sub_amcl_pose = nh_.subscribe("/summit_xl/amcl_pose", 1, &ServiceCore::cbReceiveAmclPoseSUMMIT, this);

      sub_wam_pose = nh_.subscribe("/wam/pose", 1, &ServiceCore::cbReceivePoseWAM, this);
      sub_wam_jnt_state = nh_.subscribe("/wam/joint_states", 1, &ServiceCore::cbReceiveJntStatesWAM, this);

  //  Declaration of service server for WAM operation

      srv_cart_move = nh_.serviceClient<uwarl_tasks::CartPosMove>("wam/cart_move");
      srv_ortn_move = nh_.serviceClient<uwarl_tasks::OrtnMove>("wam/ortn_move");
      srv_joint_move = nh_.serviceClient<uwarl_tasks::JointMove>("wam/joint_move");
      srv_pose_move = nh_.serviceClient<uwarl_tasks::PoseMove>("wam/pose_move");

      srv_go_home = nh_.serviceClient<std_srvs::Empty>("wam/go_home");

      srv_hand_open_grsp = nh_.serviceClient<std_srvs::Empty>("bhand/open_grasp");
      srv_hand_close_grsp = nh_.serviceClient<std_srvs::Empty>("bhand/close_grasp");
      srv_hand_open_sprd = nh_.serviceClient<std_srvs::Empty>("bhand/open_spread");
      srv_hand_close_sprd = nh_.serviceClient<std_srvs::Empty>("bhand/close_spread");

      srv_ik_wam = nh_.serviceClient<ik_solver_barrett_wam::InvKineWam>("ik_solver_wam/solve_inv_kine_wam");

      ROS_INFO("Init. System has been done");
  }



  /////////////////////////////////////////////// Callback functions /////////////////////////////////////////////////////////

  void cbCheckArrivalStatusSUMMIT(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[ROBOT_NUMBER_SUMMIT] = true;
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusSUMMIT : %d", rcvMoveBaseActionResult.status.status);
    }
  }


  void cbCheckArrivalStatusWAM(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[ROBOT_NUMBER_WAM] = true;
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusWAM : %d", rcvMoveBaseActionResult.status.status);
    }
  }

  void cbReceiveEnableAutoOperation(const std_msgs::Bool EnableAuto)
  {
    is_auto_operation_enabled_old = is_auto_operation_enabled;
    is_auto_operation_enabled = EnableAuto.data;    
    if ((is_auto_operation_enabled_old == false) && (is_auto_operation_enabled == true)) // auto-operation enabled
    { 
      ROS_INFO("Auto.Operation is on");
      
      if (is_auto_operation_reset == true)
      {  
         ROS_INFO("Auto. Operation now started");
         is_auto_operation_reset = false;
      }
    }

  }
  
  void cbReceiveMoveBaseSimpleGoal(const geometry_msgs::PoseStamped MoveBaseSimpleGoal)
  {
    poseStampedMostRecentTargetNav = MoveBaseSimpleGoal; // update the most recently received goal
    ROS_INFO("MoveBase Goal Received!!!!!!");
  }

  // When received pose of robot marker
  void cbReceivePoseRobotMarkerFromCam1(const geometry_msgs::PoseStamped CurPoseFromCam)
  {
      poseStampedCurSummitByCam1 = CurPoseFromCam;
  }
  void cbReceivePoseRobotMarkerFromCam2(const geometry_msgs::PoseStamped CurPoseFromCam)
  {
      poseStampedCurSummitByCam2 = CurPoseFromCam;
  }
  // When received pose of obj1 marker
  void cbReceivePoseObj1FromCam1(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj1ByCam1 = ObjPoseFromCam;
  }
  void cbReceivePoseObj1FromCam2(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj1ByCam2 = ObjPoseFromCam;
  }
  // When received pose of obj2 marker
  void cbReceivePoseObj2FromCam1(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj2ByCam1 = ObjPoseFromCam;
  }
  void cbReceivePoseObj2FromCam2(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj2ByCam2 = ObjPoseFromCam;
  }
  // When received pose of obj3 marker
  void cbReceivePoseObj3FromCam1(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj3ByCam1 = ObjPoseFromCam;
  }
  void cbReceivePoseObj3FromCam2(const geometry_msgs::PoseStamped ObjPoseFromCam)
  {
      poseStampedObj3ByCam2 = ObjPoseFromCam;
  }
  // When received pose of rail marker
//  void cbReceivePoseRailFromCam2(const geometry_msgs::PoseStamped ObjPoseFromCam)
//  {
//      poseRailByCam2 = ObjPoseFromCam;
//  }
  // When received pose of knob marker
  void cbReceivePoseKnobFromCam2(const geometry_msgs::PoseArray ObjPoseFromCam)
  {
      poseArrayKnobByCam2 = ObjPoseFromCam;
  }


  void cbReceiveAmclPoseSUMMIT(const geometry_msgs::PoseWithCovarianceStamped AMCL_pose)
  {
    poseWithCovarianceStampedAmcl = AMCL_pose;
  }

  void cbReceivePoseWAM(const geometry_msgs::PoseStamped WAMposeFromWAM)
  {
      poseStampedWAMpose = WAMposeFromWAM;
  }
  void cbReceiveJntStatesWAM(const sensor_msgs::JointState WAMjntState)
  {
      jointStateWAMjntState = WAMjntState;

      if(is_wam_moving){
         if(jointStateWAMjntState.position.size()>0) { // has data arrived from WAM node
             double sum;
             sum=0.0;
             for (int j=0;j<7;j++){
                 sum = sum + (jntcmd[j] - jointStateWAMjntState.position.at(j))*(jntcmd[j] - jointStateWAMjntState.position.at(j));
             }
             if (sum < 0.0001){
                     ROS_INFO("Wam reached target.");
                     is_wam_reached = true;
                     is_wam_moving = false;
             }
         }
     }
  }


  /////////////////////////////////////////////////////////////////    Customized Functions   //////////////////////////////////////////////////////////////

//  bool fnSolveInvKineWam(geometry_msgs::Pose const desPoseHand, double &jntcmd){
void fnSolveInvKineWam(){
      srvInvKineWam.request.poseToolDes = desPoseHand;
      srvInvKineWam.request.poseWamCur = poseStampedWAMpose.pose;

      if(jointStateWAMjntState.position.size()>0) { // has data arrived from WAM node
          for (int j=0;j<7;j++){
              srvInvKineWam.request.jointsCur[j] = jointStateWAMjntState.position.at(j);
          }
      } else {
          ROS_INFO("No position from WAM");
      }

      if(srv_ik_wam.call(srvInvKineWam)){
          if (srvInvKineWam.response.success == true){
              jntcmd[0] = srvInvKineWam.response.jointsSolved[0];
              jntcmd[1] = srvInvKineWam.response.jointsSolved[1];
              jntcmd[2] = srvInvKineWam.response.jointsSolved[2];
              jntcmd[3] = srvInvKineWam.response.jointsSolved[3];
              jntcmd[4] = srvInvKineWam.response.jointsSolved[4];
              jntcmd[5] = srvInvKineWam.response.jointsSolved[5];
              jntcmd[6] = srvInvKineWam.response.jointsSolved[6];

              ROS_INFO("Jnt Cmd by IK: {%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f}",jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
          }
          else
          {
              jntcmd[0] = 0;
              jntcmd[1] = -0.7;
              jntcmd[2] = 0;
              jntcmd[3] = 2.1;
              jntcmd[4] = 0;
              jntcmd[5] = 0.17;
              jntcmd[6] = 1.57;
              ROS_INFO("Jnt Cmd to default (IK failed): {%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f}",
                       jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
          }
      }

  }

  void fnGoToSafePosWam()
  {
      srvJointMove.request.joints = {0,-0.5,0,1,0,0.5,1.57};
      srv_joint_move.call(srvJointMove);
      ros::Duration(5.0).sleep();
  }

  void fnCheckModeChange()
  {
      // Check if each robot complete the task
      if(is_robot_reached_target[ROBOT_NUMBER_SUMMIT] == true)
      {
        is_cur_mode_completed = true;
        is_robot_reached_target[ROBOT_NUMBER_SUMMIT] = false;
      }
      else if(is_robot_reached_target[ROBOT_NUMBER_WAM] == true)
      {
        // is_cur_mode_completed = true;
        is_robot_reached_target[ROBOT_NUMBER_WAM] = false;
      }

      if(is_cur_mode_completed)
      {
        flag_go_to_next_mode = true; // flag on when the current mode is completed 
        // ROS_INFO(" ----  Cur.Mode(%d) has been compledted ---- ", mode_thistime);
      }
      is_cur_mode_completed = false;
  }

  void fnSendJntCmdWAM(float cmd1,float cmd2,float cmd3,float cmd4,float cmd5,float cmd6,float cmd7)
  {
      srvJointMove.request.joints = {cmd1,cmd2,cmd3,cmd4,cmd5,cmd6,cmd7};
      srv_joint_move.call(srvJointMove);

      is_wam_moving = true;
      is_wam_reached = false;
      ROS_INFO("Jnt.Cmd has been sent to WAM,[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]", cmd1,cmd2,cmd3,cmd4,cmd5,cmd6,cmd7);
      ros::Duration(10.0).sleep();
  }

  void fnDoActionWhenModeChange()
  {
    // action happens only if "auto.operation" is enabled
    if (is_auto_operation_enabled)
    {
      if (mode_thistime == WAIT)
      {
        flag_go_to_next_mode = true;
      }

      // mode change to the next step 
      if (flag_go_to_next_mode)
      {
        mode_thistime = mode_thistime+1;
        if (mode_thistime > NUM_OF_MODES)
        {
            mode_thistime = MOVE_TO_TABLE_A_FROM_ANYWHERE;
        }

        switch (mode_thistime) // do action for the corresponding mode
        {
            case MOVE_TO_TABLE_A_FROM_ANYWHERE:
                // go to Table A
                pubPoseStampedSummit.publish(poseStampedTargetNav[TABLE_A]);
                ROS_INFO("[Mode 01] Aim: Move to Table A");
                flag_go_to_next_mode = false; // reset the flag after the action

//                ros::Duration(10.0).sleep();
//                ROS_INFO("10 sec wait");

//                is_cur_mode_completed = true;
                break;
            case PICK_BOTTLE_FROM_A:
                ros::Duration(3.0).sleep();
                ROS_INFO("[Mode 02] Aim: Pick a bottle from Table A");
                flag_go_to_next_mode = false;

//1. Get Rel.Pose & Compute desired pose, on picking table (ZED1)
                tf2::fromMsg(poseStampedCurSummitByCam1.pose,tfCamToRobotMarker);
                tf2::fromMsg(poseStampedObj1ByCam1.pose,tfCamToObj1);
                tfWamToObj1 = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToObj1)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToObj1,poseObj1FromWAM);

                pubPoseObj1FromWAM.publish(poseObj1FromWAM);

                ROS_INFO("Obj From WAM: {x=%.4f,y=%.4f,z=%.4f,qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f}",poseObj1FromWAM.position.x,
                poseObj1FromWAM.position.y, poseObj1FromWAM.position.z,
                poseObj1FromWAM.orientation.x,poseObj1FromWAM.orientation.y,poseObj1FromWAM.orientation.z,poseObj1FromWAM.orientation.w);
                // orientation
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseObj1FromWAM.position.y,poseObj1FromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseObj1FromWAM.position;
                desPoseHand.position.z = desPoseHand.position.z - 0.05;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK
                fnSolveInvKineWam();
// 3. go to safe position (sleep(5.0) is in)
                fnGoToSafePosWam();
// 4. Open "hand
                srv_hand_open_grsp.call(srvCmdEmpty);
//                ros::Duration(2.0).sleep();
// 5. move to object (10s wait included)
                fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
// 6. Grasp hand
                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 7. Go back to safe position
                fnGoToSafePosWam();

                is_cur_mode_completed = true;
                // // Flag is not reset at this moment so the mode can be move forward without an action, TBD
                break;
            case MOVE_TO_TABLE_B_FROM_A:
                pubPoseStampedSummit.publish(poseStampedTargetNav[TABLE_B]);
                ROS_INFO("[Mode 03] Aim: Move to Table B");
                flag_go_to_next_mode = false; // reset the flag after the action
                break;
            case PLACE_BOTTLE_ON_B:
                ros::Duration(3.0).sleep();
                ROS_INFO("[Mode 04] Aim: Place the bottle on Table B");
                flag_go_to_next_mode = false;

// 1. Get Rel.pose & Compute deisred pose of rail wrt WAM base, on placing table (ZED2)
                tf2::fromMsg(poseStampedCurSummitByCam2.pose,tfCamToRobotMarker);
                tf2::fromMsg(poseRailByCam2,tfCamToRail);
                tfWamToRail = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToRail)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToRail,poseRailFromWAM);

                pubPoseRailFromWAM.publish(poseRailFromWAM);
                ROS_INFO("Rail From WAM: {x=%.4f,y=%.4f,z=%.4f,qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f}",poseRailFromWAM.position.x,
                         poseRailFromWAM.position.y, poseRailFromWAM.position.z,
                         poseRailFromWAM.orientation.x,poseRailFromWAM.orientation.y,poseRailFromWAM.orientation.z,poseRailFromWAM.orientation.w);
                // orientation
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseRailFromWAM.position.y,poseRailFromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseRailFromWAM.position;
                desPoseHand.position.z = poseRailFromWAM.position.z - 0.25;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK
                fnSolveInvKineWam();
// 3. go to safe position (sleep(5.0) is in)
                fnGoToSafePosWam();
// 4. move to obejct Send joint cmds to WAM (10s wait included)
                fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
// 5. Open hand
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 6. Go back to safe position
                fnGoToSafePosWam();
// 7. Grasp hand
                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();

                is_cur_mode_completed = true;
              break;
            case MOVE_TO_KNOB:
                pubPoseStampedSummit.publish(poseStampedTargetNav[KNOB]);
                ROS_INFO("[Mode 05] Aim: Move to Knob");
                flag_go_to_next_mode = false; // reset the flag after the action
//                is_cur_mode_completed = true;
                break;
            case TIGHTEN_KNOB:
                // wait until the movement becomes stationary so the camera can read the data
                ros::Duration(3.0).sleep();

                ROS_INFO("[Mode 06] Aim: Tighten the knob");
                flag_go_to_next_mode = false;
// 1. Get Rel.Pose & Compute desired pose
                tf2::fromMsg(poseStampedCurSummitByCam2.pose,tfCamToRobotMarker);
                tf2::fromMsg(poseArrayKnobByCam2.poses[0],tfCamToKnob);  //need to check the variable type (poseArray.pose[])
                tfWamToKnob = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToKnob)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToKnob,poseKnobFromWAM);
                // orientation
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseRailFromWAM.position.y,poseRailFromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseKnobFromWAM.position;
                desPoseHand.position.z = poseKnobFromWAM.position.z - 0.05;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK
                fnSolveInvKineWam();
// 3. open hand
                srv_hand_open_grsp.call(srvCmdEmpty);
//                ros::Duration(2.0).sleep();
// 4. Send joint cmds to WAM (10s wait included)
                fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
// 5. grasp close
//                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();

// 6. WAM move the trajectory (TBD)

// 7. grasp open
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 8. back to safe pose
                fnGoToSafePosWam();

                is_cur_mode_completed = true;
                // Flag is not reset at this moment so the mode can be move forward without an action, TBD
                break;
            case RELEASE_KNOB:
                ROS_INFO("[Mode 07] Aim: Release the knob");
                flag_go_to_next_mode = false;

// 1. Get the desired target
                tf2::fromMsg(poseStampedCurSummitByCam2.pose,tfCamToRobotMarker);
//                tf2::fromMsg(poseArrayKnobByCam2.poses[0],tfCamToKnob);  //need to check the variable type (poseArray.pose[])
                tf2::fromMsg(poseKnobByCam2,tfCamToKnob);  //need to check the variable type (poseArray.pose[])
                tfWamToKnob = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToKnob)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToKnob,poseKnobFromWAM);

                pubPoseKnobFromWAM.publish(poseKnobFromWAM);
                ROS_INFO("Knob From WAM: {x=%.4f,y=%.4f,z=%.4f,qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f}",poseKnobFromWAM.position.x,
                         poseKnobFromWAM.position.y, poseKnobFromWAM.position.z,
                         poseKnobFromWAM.orientation.x,poseKnobFromWAM.orientation.y,poseKnobFromWAM.orientation.z,poseKnobFromWAM.orientation.w);
                // orientation
                //                Bhand_grasp_ortn.setRPY(1.278,0.0,0.0);
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseRailFromWAM.position.y,poseRailFromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseKnobFromWAM.position;
                desPoseHand.position.z = poseKnobFromWAM.position.z - 0.05;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK
                fnSolveInvKineWam();
// 4. Send joint cmds to WAM (10s wait included)
                fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
// 5. grasp close
//                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();

// 6. WAM move the trajectory (TBD)

// 7. grasp open
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 8. back to safe pose
                fnGoToSafePosWam();
// 9. wam grasp close
                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();

                is_cur_mode_completed = true;
                // Flag is not reset at this moment so the mode can be move forward without an action, TBD
                break;
            case MOVE_TO_TABLE_B_FROM_KNOB:
                pubPoseStampedSummit.publish(poseStampedTargetNav[TABLE_B]);
                 ROS_INFO("[Mode 08] Aim: Move to Table B from Knob");
                flag_go_to_next_mode = false; // reset the flag after the action
                break;
            case PICK_BOTTLE_FROM_B:
            // wait movement stops
               ros::Duration(3.0).sleep();
                 ROS_INFO("[Mode 09] Aim: Pick the bottle from Table B");
                flag_go_to_next_mode = false;

// 1. Get the Rel.Pose & Compute target pose
                tf2::fromMsg(poseStampedCurSummitByCam2.pose,tfCamToRobotMarker);
                tf2::fromMsg(poseStampedObj1ByCam2.pose,tfCamToObj1);
                tfWamToObj1 = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToObj1)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToObj1,poseObj1FromWAM);

                pubPoseObj1FromWAM.publish(poseObj1FromWAM);
                ROS_INFO("Obj From WAM: {x=%.4f,y=%.4f,z=%.4f,qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f}",poseObj1FromWAM.position.x,
                         poseObj1FromWAM.position.y, poseObj1FromWAM.position.z,
                         poseObj1FromWAM.orientation.x,poseObj1FromWAM.orientation.y,poseObj1FromWAM.orientation.z,poseObj1FromWAM.orientation.w);

                // Grasping orientation: straight toward the object
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseObj1FromWAM.position.y,poseObj1FromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseObj1FromWAM.position;
                desPoseHand.position.z = poseObj1FromWAM.position.z - 0.05;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK
                fnSolveInvKineWam();
// 3. open grasp
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 4. Send the jount cmd: Go to the object
                 fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);

// 5. close grasp
                srv_hand_close_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 6. go back to safe pose
                fnGoToSafePosWam();

                is_cur_mode_completed = true;
                // Flag is not reset at this moment so the mode can be move forward without an action, TBD
                break;
            case MOVE_TO_TABLE_A_FROM_B:
                pubPoseStampedSummit.publish(poseStampedTargetNav[TABLE_A]);
                ROS_INFO("[Mode 10] Aim: Move back to Table A");
                flag_go_to_next_mode = false; // reset the flag after the action
//                ros::Duration(10.0).sleep();
//                ROS_INFO("10 sec wait");
//                is_cur_mode_completed = true;
                break;  
            case PLACE_BOTTLE_ON_A:
                ros::Duration(3.0).sleep();
                ROS_INFO("[Mode 11] Aim: Place the bottle back on Table A");
                flag_go_to_next_mode = false;

// 1. Get the Rel.Pose & Compute desired pose
                tf2::fromMsg(poseStampedCurSummitByCam1.pose,tfCamToRobotMarker);
                tf2::fromMsg(poseObj1OnTableACam1,tfCamToObj1);
                tfWamToObj1 = tfWamToRobotMarker.operator*(tfCamToRobotMarker.inverseTimes(tfCamToObj1)); // wTo = wTm*mTo = wTm*(mTc*cTo) = wTm*(inv(cTm)*cTo)
                tf2::toMsg(tfWamToObj1,poseObj1FromWAM);

                pubPoseObj1FromWAM.publish(poseObj1FromWAM);
                ROS_INFO("Obj From WAM: {x=%.4f,y=%.4f,z=%.4f,qx=%.4f,qy=%.4f,qz=%.4f,qw=%.4f}",poseObj1FromWAM.position.x,
                         poseObj1FromWAM.position.y, poseObj1FromWAM.position.z,
                         poseObj1FromWAM.orientation.x,poseObj1FromWAM.orientation.y,poseObj1FromWAM.orientation.z,poseObj1FromWAM.orientation.w);

                // Grasping orientation: straight toward the object
                yaw_for_grasp = BHAND_GRASP_ORNT_YAW + atan2(poseObj1FromWAM.position.y,poseObj1FromWAM.position.x);
                Bhand_grasp_ortn.setRPY(BHAND_GRASP_ORNT_ROLL,BHAND_GRASP_ORNT_PITCH,yaw_for_grasp);
                quatBHandGraspObjOrtn = tf2::toMsg(Bhand_grasp_ortn);
                // desired pose
                desPoseHand.position = poseObj1FromWAM.position;
                desPoseHand.position.z = poseObj1FromWAM.position.z;
                desPoseHand.orientation = quatBHandGraspObjOrtn;

// 2. Solve IK WAM
                fnSolveInvKineWam();
// 3. Send joint command
                fnSendJntCmdWAM(jntcmd[0],jntcmd[1],jntcmd[2],jntcmd[3],jntcmd[4],jntcmd[5],jntcmd[6]);
// 4. open grasp (placing)
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
// 5. wam move back to safe pose
                fnGoToSafePosWam();
// 6. wam grasp close
                srv_hand_open_grsp.call(srvCmdEmpty);
                ros::Duration(2.0).sleep();
                is_cur_mode_completed = true;
                // Flag is not reset at this moment so the mode can be move forward without an action, TBD
              break;                          

        }
      }
    }
  }


///////////////////////////////////         Declarations      /////////////////////////////////////////////////////////////////

private:

  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pubServiceStatusSummit;
  ros::Publisher pubServiceStatusWAM;

  ros::Publisher pubPoseStampedSummit;
  ros::Publisher pubPoseStampedWAM;

  ros::Publisher pubPoseObj1FromWAM;
  ros::Publisher pubPoseObj2FromWAM;
  ros::Publisher pubPoseObj3FromWAM;
  ros::Publisher pubPoseRailFromWAM;
  ros::Publisher pubPoseKnobFromWAM;

  // Subscriber
  ros::Subscriber sub_enable_auto_operation;

  ros::Subscriber sub_robot_marker_pose_by_cam1, sub_robot_marker_pose_by_cam2;
  ros::Subscriber sub_obj1_pose_by_cam1, sub_obj1_pose_by_cam2;
  ros::Subscriber sub_obj2_pose_by_cam1, sub_obj2_pose_by_cam2;
  ros::Subscriber sub_obj3_pose_by_cam1, sub_obj3_pose_by_cam2;
//  ros::Subscriber sub_rail_pose_by_cam2;
  ros::Subscriber sub_knob_pose_by_cam2;

  ros::Subscriber sub_move_base_simple_goal;
  ros::Subscriber sub_amcl_pose;

  ros::Subscriber sub_arrival_status_summit;
  ros::Subscriber sub_arrival_status_wam;
  ros::Subscriber sub_arrival_status_camera;

  ros::Subscriber sub_wam_pose;
  ros::Subscriber sub_wam_jnt_state;

  // Service Clients (for requests(i.eservice call) to WAM)
  ros::ServiceClient srv_cart_move, srv_ortn_move, srv_pose_move, srv_joint_move, srv_go_home; 
  ros::ServiceClient srv_hand_close_grsp, srv_hand_open_grsp;
  ros::ServiceClient srv_hand_close_sprd, srv_hand_open_sprd;   

  // Service Clients for Inv.Kinematics of WAM
  ros::ServiceClient srv_ik_wam;

  // msgs for target poses
  geometry_msgs::PoseStamped poseStampedTargetNav[3]; // Table A, Table B, knob
//  geometry_msgs::PoseStamped poseStampedTargetWAM[2]; // # of array to be determined
//  geometry_msgs::Pose poseTargetNavByCam[3]; // Desired pose in the camera view
  geometry_msgs::Pose poseTargetNavByCamThisTime; // Desired pose in the camera view
  geometry_msgs::PoseStamped poseFineAdjustInAmclFrame;
  geometry_msgs::PoseStamped poseStampedCurSummitByCam1,poseStampedCurSummitByCam2; // Current pose in the camera view
  geometry_msgs::PoseStamped poseStampedObj1ByCam1,poseStampedObj1ByCam2;       // pose of object1 received from cameras
  geometry_msgs::PoseStamped poseStampedObj2ByCam1,poseStampedObj2ByCam2;       // pose of object2 received from cameras
  geometry_msgs::PoseStamped poseStampedObj3ByCam1,poseStampedObj3ByCam2;       // pose of object3 received from cameras
  geometry_msgs::PoseArray poseArrayKnobByCam2;                                 // pose of knob received from camera 2

  geometry_msgs::PoseStamped poseStampedMostRecentTargetNav; // most recently saved target for navigation
  geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStampedAmcl; //AMCL_pose from summit

  // Transformation of each from camera (in order in the tasks)
  tf2::Transform tfCamToRobotMarker;
  tf2::Transform tfCamToObj1;
  tf2::Transform tfCamToObj2;
  tf2::Transform tfCamToObj3;
  tf2::Transform tfCamToRail;
  tf2::Transform tfCamToKnob;

  tf2::Transform tfWamToObj1;
  tf2::Transform tfWamToObj2;
  tf2::Transform tfWamToObj3;
  tf2::Transform tfWamToRail;
  tf2::Transform tfWamToKnob;

// Static Transformation between Wam and Robot_marker (Wam -> RobotMarker)
  tf2::Transform tfWamToRobotMarker;
// Static Poses (rails, placing locations)
  geometry_msgs::Pose poseObj1OnTableACam1;                           // pose of place of obj1 received from camera 2
  geometry_msgs::Pose poseObj2OnTableACam1;                             // pose of rail received from camera 2
  geometry_msgs::Pose poseObj3OnTableACam1;                             // pose of rail received from camera 2
  geometry_msgs::Pose poseRailByCam2;                             // pose of rail received from camera 2
  geometry_msgs::Pose poseKnobByCam2;                             // pose of rail received from camera 2

// (Relative) Pose of each target from WAM base
  geometry_msgs::Pose poseObj1FromWAM;
  geometry_msgs::Pose poseObj2FromWAM;
  geometry_msgs::Pose poseObj3FromWAM;
  geometry_msgs::Pose poseRailFromWAM;
  geometry_msgs::Pose poseKnobFromWAM;

 // for wam
  uwarl_tasks::CartPosMove  srvCartMove;
  uwarl_tasks::OrtnMove  srvOrtnMove;
  uwarl_tasks::PoseMove  srvPoseMove;
  uwarl_tasks::JointMove  srvJointMove;

  // ik_solver of the wam
  ik_solver_barrett_wam::InvKineWam srvInvKineWam;
  tf2::Quaternion Bhand_grasp_ortn;
  geometry_msgs::Quaternion quatBHandGraspObjOrtn;

  std_srvs::Empty srvCmdEmpty;

  std::vector<double> target_pose_position;
  std::vector<double> target_pose_orientation;

  geometry_msgs::PoseStamped poseStampedWAMpose;        // pose of the end-effector of WAM received from WAM
  sensor_msgs::JointState jointStateWAMjntState;

  bool is_robot_reached_target[2] = {true, true};
  bool is_wam_moving = false;
  double cur_target_jnt[7];
  bool is_wam_reached = false;

  bool is_cur_mode_completed = true;
  bool flag_go_to_next_mode = false;

  int  mode_thistime = 0; 
  bool is_auto_operation_enabled = false;
  bool is_auto_operation_enabled_old = false;
  bool is_auto_operation_reset = true;

  int count_interval_fine_goal = 0;

  float des_pos_x,des_pos_y,des_pos_z;
  float des_ornt_x,des_ornt_y,des_ornt_z,des_ornt_w;

  float jntcmd[7];

  double yaw_for_grasp;

  geometry_msgs::Pose desPoseHand;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "service_core_expo");
  std::cout << "e";

  //Create an object of class ServiceCore that will take care of everything
  ServiceCore serviceCore;

  ros::spin();

  return 0;
}
