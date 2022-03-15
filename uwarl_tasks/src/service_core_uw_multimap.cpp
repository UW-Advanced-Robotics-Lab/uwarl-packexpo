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

#include "multimap_server_msgs/PubMap.h"

#include "uwarl_tasks/KnobOperation.h"

#include <std_srvs/Empty.h>

#include <math.h>

#define ROBOT_NUMBER_SUMMIT 0
#define ROBOT_NUMBER_WAM 1

#define WAM_DOF 7

// Targets for Navigation
#define TABLE_A 0
#define TABLE_B 1
#define KNOB 2
#define ROBOHUB_FRONT 3

#define CORRIDOR_DOOR 0
#define ELEV_FRONT 1
#define ELEV_INSIDE 2
#define ROBOHUB_FRONT 3

// mode
#define WAIT 0
#define MOVE_TO_DOOR_BUTTON_FROM_LAB 1
#define PRESS_DOOR_BUTTON 2
#define MOVE_TO_ELEV_FRONT_FROM_DOOR_BUTTON 3
#define PRESS_ELEV_CALL_BUTTON 4
#define MOVE_TO_ELEV_INSIDE_FROM_ELEV_FRONT 5
#define PRESS_ELEV_FLOOR_BUTTON 6
#define CHANGE_MAP 7
#define MOVE_TO_ROBOHUB_FRONT_FROM_ELEV_INSIDE 8

#define FLAG_CUR_MODE_FAIL -1
#define FLAG_MOVE_TO_DOOR_BUTTON_FROM_LAB 1
#define FLAG_PRESS_DOOR_BUTTON 1
#define FLAG_MOVE_TO_ELEV_FRONT_FROM_DOOR_BUTTON 2
#define FLAG_PRESS_ELEV_CALL_BUTTON 2
#define FLAG_MOVE_TO_ELEV_INSIDE_FROM_ELEV_FRONT 3
#define FLAG_PRESS_ELEV_FLOOR_BUTTON 3
#define FLAG_CHANGE_MAP 4
#define FLAG_MOVE_TO_ROBOHUB_FRONT_FROM_ELEV_INSIDE 5

#define NUM_OF_MODES 8

#define COUNT_FINE_NAV 30

//static int NUM_OF_MAPS;
//static int NUM_OF_NAV_TARGETS;

#define NUM_OF_MAPS 2
#define NUM_OF_NAV_TARGETS 4

const float PI = 3.141592653589793;

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

// Knob parameter
const double knobAngleDeg = 20.0; // angle(deg) of knob
const double offsetKnobZ = -0.05; // offset of grasping point along the axis of knob, 5cm below from the knob's top

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
     std::string param_name;

    // SUMMIT: Get the target poses for navigation: Corridor door, Elevator front, Elevator inside, RoboHub front
//    nh_.getParam("num_of_nav_targets", NUM_OF_NAV_TARGETS);
    for (int i = 0; i < NUM_OF_NAV_TARGETS; i++)
    { 
        param_name = "nav_target_" + std::to_string(i) + "/position";
        nh_.getParam(param_name, read_pose_position);
        param_name = "nav_target_" + std::to_string(i) + "/orientation";
        nh_.getParam(param_name, read_pose_orientation);

        poseStampedTargetNav[i].header.frame_id = "summit_xl_map";
        poseStampedTargetNav[i].header.stamp = ros::Time::now();
     
        poseStampedTargetNav[i].pose.position.x = read_pose_position[0];
        poseStampedTargetNav[i].pose.position.y = read_pose_position[1];
        poseStampedTargetNav[i].pose.position.z = read_pose_position[2];

        poseStampedTargetNav[i].pose.orientation.x = read_pose_orientation[0];
        poseStampedTargetNav[i].pose.orientation.y = read_pose_orientation[1];
        poseStampedTargetNav[i].pose.orientation.z = read_pose_orientation[2];
        poseStampedTargetNav[i].pose.orientation.w = read_pose_orientation[3];
    }
 
    // map origins
    // number of maps
//    nh_.getParam("num_of_maps", NUM_OF_MAPS);
    for (int i = 0; i < NUM_OF_MAPS; i++)
    { 
        param_name = "map" + std::to_string(i) + "_origin/position";
        nh_.getParam(param_name, read_pose_position);
        param_name = "map" + std::to_string(i) + "_origin/orientation";
        nh_.getParam(param_name, read_pose_orientation);

        poseMapOrigin[i].position.x = read_pose_position[0];
        poseMapOrigin[i].position.y = read_pose_position[1];
        poseMapOrigin[i].position.z = read_pose_position[2];

        poseMapOrigin[i].orientation.x = read_pose_orientation[0];
        poseMapOrigin[i].orientation.y = read_pose_orientation[1];
        poseMapOrigin[i].orientation.z = read_pose_orientation[2];
        poseMapOrigin[i].orientation.w = read_pose_orientation[3];
    }
    
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

    // to initialize summit's pose inside elevator when the map is changed to E7-1st floor
    nh_.getParam("map1_init_pose_inside_elev/position", read_pose_position);
    nh_.getParam("map1_init_pose_inside_elev/orientation", read_pose_orientation);

    poseWithCovStampedInitialpose.pose.pose.position.x = read_pose_position[0];
    poseWithCovStampedInitialpose.pose.pose.position.y = read_pose_position[1];
    poseWithCovStampedInitialpose.pose.pose.position.z = read_pose_position[2];

    poseWithCovStampedInitialpose.pose.pose.orientation.x = read_pose_orientation[0];
    poseWithCovStampedInitialpose.pose.pose.orientation.y = read_pose_orientation[1];
    poseWithCovStampedInitialpose.pose.pose.orientation.z = read_pose_orientation[2];
    poseWithCovStampedInitialpose.pose.pose.orientation.w = read_pose_orientation[3];

    nh_.getParam("map1_init_pose_inside_elev/covariance", read_values);
//    poseWithCovStampedInitialpose.pose.covariance = {0.030233276987871704, 0.0008451303042420477, 0.0, 0.0, 0.0, 0.0, 0.0008451303042420477, 0.0244127803076708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.022876343366175073};

    for (int i=0; i<36; i++){
    poseWithCovStampedInitialpose.pose.covariance[i] = read_values[i];        
    }

    completion_ind_step_wam.data = 0;
    completion_ind_step_summit.data = 0;

    mode_thistime = WAIT;
    mode_nexttime = MOVE_TO_DOOR_BUTTON_FROM_LAB;

    ROS_INFO("Init. Param has been done");
  }

  void fnSystemInit()
  {
      ROS_INFO("Init. System. started");

      pubPoseStampedSummit = nh_.advertise<geometry_msgs::PoseStamped>("/summit_xl/move_base_simple/goal", 1);
      // pubPoseStampedWAM = nh_.advertise<geometry_msgs::PoseStamped>("/wam/move_base_simple/goal", 1);
      pubPoseWithCovStampedInitialpose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("summit_xl/initialpose",1);

      pubPoseObj1FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj1", 1);
      pubPoseObj2FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj2", 1);
      pubPoseObj3FromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/obj3", 1);
      pubPoseRailFromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/rail", 1);
      pubPoseKnobFromWAM  = nh_.advertise<geometry_msgs::Pose>("/uwarl/pose_from_wam/knob", 1);

      sub_enable_auto_operation = nh_.subscribe("/enable_auto_operation", 1, &ServiceCore::cbReceiveEnableAutoOperation, this);
      
      // Completion flag for subtasks (summit, WAM)
      sub_arrival_status_summit = nh_.subscribe("/summit_xl/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusSUMMIT, this);
      sub_arrival_status_wam = nh_.subscribe("/task_completion_flag_wam", 1, &ServiceCore::cbCheckArrivalStatusWAM, this);

      pubArrivalStatusSummit = nh_.advertise<std_msgs::Int8>("/task_completion_flag_summit", 1);
      rosBoolTrue.data = true;
      completion_ind_step_wam.data = 0;
      completion_ind_step_summit.data = 0;
      // pubArrivalStatusWAM = nh_.advertise<std_msgs::String>("/task_completion_flag_wam", 1);

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


  //  Declaration of service cleint for multimap_server
      srv_map_change = nh_.serviceClient<multimap_server_msgs::PubMap>("summit_xl/multimap_server/pub_map_with_name");
//      srv_map_change = nh_.serviceClient<multimap_server_msgs::PubMap>("multimap_server/pub_map_with_name");
  //  Declaration of service cleint for clear_costmap
      srv_clear_costmap = nh_.serviceClient<std_srvs::Empty>("summit_xl/move_base/clear_costmap");

  //  Declaration of service client for WAM operation

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

      srv_knob_operation_wam = nh_.serviceClient<uwarl_tasks::KnobOperation>("knob_operation_wam/send_jntcmd_knob_operation");

      ROS_INFO("Init. System has been done");
  }



  /////////////////////////////////////////////// Callback functions /////////////////////////////////////////////////////////

  void cbCheckArrivalStatusSUMMIT(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3) // 3 is the "goal-reached" status
    {
      is_robot_reached_target[ROBOT_NUMBER_SUMMIT] = true;
    }
    else if (rcvMoveBaseActionResult.status.status == 4) // 4 is the "Failed to find a path" 
    {      
      ROS_INFO("[Nav_Error] failed to find a valid path. Planning will be repeated.");
      mode_nexttime = mode_thistime; // next mode will be the current mode to repeat
      is_robot_reached_target[ROBOT_NUMBER_SUMMIT] = true; // dummy flag to restart the current mode as if it is newly set
      completion_ind_step_summit.data = -1; // yet, the flag indicates a failure of current mode
    }
    else if (rcvMoveBaseActionResult.status.status != 1)
    {
      ROS_INFO("cbCheckArrivalStatusSUMMIT : %d", rcvMoveBaseActionResult.status.status);
      completion_ind_step_summit.data = -1;
    }
  }


  void cbCheckArrivalStatusWAM(const std_msgs::Int8 rcvFlagWamCompletion)
  {
    completion_ind_step_wam.data = rcvFlagWamCompletion.data;
    switch(completion_ind_step_wam.data)
    {
      case FLAG_PRESS_DOOR_BUTTON:
        mode_thistime = PRESS_DOOR_BUTTON;
        mode_nexttime = MOVE_TO_ELEV_FRONT_FROM_DOOR_BUTTON;
        is_robot_reached_target[ROBOT_NUMBER_WAM] = true;
      break;

      case FLAG_PRESS_ELEV_CALL_BUTTON:
        mode_thistime = PRESS_ELEV_CALL_BUTTON;
        mode_nexttime = MOVE_TO_ELEV_INSIDE_FROM_ELEV_FRONT;     
        is_robot_reached_target[ROBOT_NUMBER_WAM] = true;
      break;

      case FLAG_PRESS_ELEV_FLOOR_BUTTON:
        mode_thistime = PRESS_ELEV_FLOOR_BUTTON;
        mode_nexttime = CHANGE_MAP;     
        is_robot_reached_target[ROBOT_NUMBER_WAM] = true;
      break;

      case FLAG_CUR_MODE_FAIL:
        ROS_INFO("Current WAM operation failed: %d", rcvFlagWamCompletion.data);
      break;
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
      for (int j=0;j<7;j++){
          jntact[j] = jointStateWAMjntState.position.at(j);
      }

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
void fnCompGraspPoseKnob(){

   tf2::Vector3 zAxisKnobInWAMframe, positionGrasp;
   tf2::Matrix3x3 Rx1,Ry,Rx2,R;
   tf2::Transform wTgrasp;
   double phi,psi;

//   tf2::Vector3 positionKnob = {0.4,0.5,0.6};
//   tf2::Quaternion orientationKnob = {0.15,-0.2109,0.0,0.9659};
//   tfWamToKnob.setOrigin(positionKnob);
//   tfWamToKnob.setRotation(orientationKnob);

   //1. Obtain the direction of z-axis of knob wrt WAM frame
   zAxisKnobInWAMframe =  tfWamToKnob.getBasis().getColumn(2);
//   ROS_INFO("n_knob:%.4f,%.4f,%.4f",zAxisKnobInWAMframe[0],zAxisKnobInWAMframe[1],zAxisKnobInWAMframe[2]);

   //2. Compute the rotation angles that will be applied to moving frame of hand_grasp_pose, wam_R_grasp = Rx'(90deg)Ry'(phi)Rx'(psi) (x1-y-x2 intrinsic rotation)
   phi = atan2(zAxisKnobInWAMframe[1],zAxisKnobInWAMframe[0])-1.570796326794897;
   ROS_INFO("phi:%.4f",phi);
   psi = -knobAngleDeg*PI/180; //deg*(pi/180)
   Rx1.setRPY(PI/2,0,0);

//   for (int j=0;j<3;j++){
//       ROS_INFO("RotationToKnob:%.4f,%.4f,%.4f",Rx1.getRow(j)[0],Rx1.getRow(j)[1],Rx1.getRow(j)[2]);
//   }

   Ry.setRPY(0,phi,0);
   Rx2.setRPY(psi,0,0);
   R = Rx1*Ry*Rx2; // intrinsic rotation, post-multiplying

   for (int j=0;j<3;j++){
          ROS_INFO("RotationToKnob:%.4f,%.4f,%.4f",R.getRow(j)[0],R.getRow(j)[1],R.getRow(j)[2]);
      }

   //3. get the tf for grasping by setting the orientation by above R
   wTgrasp.setBasis(R);  // orientation
   positionGrasp = tfWamToKnob.getOrigin() + zAxisKnobInWAMframe*offsetKnobZ;
   wTgrasp.setOrigin(positionGrasp);
   tf2::toMsg(wTgrasp, poseForGraspKnob);
   ROS_INFO("grasping target:%.4f,%.4f,%.4f,   %.4f,%.4f,%.4f,%.4f",poseForGraspKnob.position.x,poseForGraspKnob.position.y,poseForGraspKnob.position.z
            ,poseForGraspKnob.orientation.x,poseForGraspKnob.orientation.y,poseForGraspKnob.orientation.z,poseForGraspKnob.orientation.w);
}

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
        is_cur_mode_completed = true;
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
/*
        mode_thistime = mode_thistime+1;
        if (mode_thistime > NUM_OF_MODES)
        {
            mode_thistime = MOVE_TO_DOOR_BUTTON_FROM_LAB;
        }
*/
        mode_thistime = mode_nexttime;

        switch (mode_thistime) // do action for the corresponding mode
        {
            case MOVE_TO_DOOR_BUTTON_FROM_LAB: // SUMMIT operation
                // go to the corridor door
                flag_go_to_next_mode = false; // reset the flag
                completion_ind_step_summit.data = 1;
                ROS_INFO("[Mode 01] Aim: Move to the corridor door");
                pubPoseStampedSummit.publish(poseStampedTargetNav[CORRIDOR_DOOR]); // send a navigation goal to summit

                mode_nexttime = PRESS_DOOR_BUTTON;
//                ros::Duration(1.0).sleep(); // for test 
//                is_cur_mode_completed = true; // for test
                 
                // wait until the completion of navigation which is checked by a callback function (cbCheckArrivalStatiusSummit)
                break;
            case PRESS_DOOR_BUTTON: // WAM operation
                flag_go_to_next_mode = false; // reset the flag
                ROS_INFO("[Mode 02] Aim: Press the door button to open the door");

                ros::Duration(0.5).sleep();

                completion_ind_step_summit.data = 1;
                pubArrivalStatusSummit.publish(completion_ind_step_summit); // Send the flag of summit completion to WAM
                mode_nexttime = MOVE_TO_ELEV_FRONT_FROM_DOOR_BUTTON;

                // expecting WAM to perform its task and send back its completion flag. 
                // Once the flag of WAM completion is received, a callback function (cbCheckArrivalStatiusSummit) is called

                break;
            case MOVE_TO_ELEV_FRONT_FROM_DOOR_BUTTON: // SUMMIT operation
                flag_go_to_next_mode = false; // reset the flag
                ROS_INFO("[Mode 03] Aim: Move to the front of the elevator");
                if (completion_ind_step_wam.data == 1)
                {
                    completion_ind_step_summit.data = 2;
                    pubPoseStampedSummit.publish(poseStampedTargetNav[ELEV_FRONT]);
//                    ros::Duration(1.0).sleep(); // for test 
//                    is_cur_mode_completed = true; // for test                   
                }
                else
                {
                  ROS_INFO("not received a proper completion task from WAM");
                }
                mode_nexttime = PRESS_ELEV_CALL_BUTTON;
                break;
            case PRESS_ELEV_CALL_BUTTON: // WAM operation
                flag_go_to_next_mode = false;
                ROS_INFO("[Mode 04] Aim: Press the elevator's call button");

                ros::Duration(0.5).sleep();

                completion_ind_step_summit.data = 2;
                pubArrivalStatusSummit.publish(completion_ind_step_summit); // Send the flag of summit completion to WAM

                mode_nexttime = MOVE_TO_ELEV_INSIDE_FROM_ELEV_FRONT;
              break;
            case MOVE_TO_ELEV_INSIDE_FROM_ELEV_FRONT: // SUMMIT operation
                flag_go_to_next_mode = false; // reset the flag
                ROS_INFO("[Mode 05] Aim: Move to inside the elevator");
                if (completion_ind_step_wam.data == 2)
                {
                    completion_ind_step_summit.data = 3;

                    srv_clear_costmap.call(srvCmdEmpty);
                    ros::Duration(0.5).sleep(); // wait for clearing costmap 
                    pubPoseStampedSummit.publish(poseStampedTargetNav[ELEV_INSIDE]);
//                      ros::Duration(2.0).sleep(); // for test 
//                      is_cur_mode_completed = true; // for test                    
                 } 
                else
                {
                  ROS_INFO("not received a proper completion task from WAM");
                }

                mode_nexttime = PRESS_ELEV_FLOOR_BUTTON;
              break;
            case PRESS_ELEV_FLOOR_BUTTON: // WAM operation
                flag_go_to_next_mode = false;
                ROS_INFO("[Mode 06] Aim: Press the elevator floor button");

                ros::Duration(0.5).sleep();

                completion_ind_step_summit.data = 3;
                pubArrivalStatusSummit.publish(completion_ind_step_summit); // Send the flag of summit completion to WAM

                mode_nexttime = CHANGE_MAP;
                break;
            case CHANGE_MAP: // map change while the robot  
                flag_go_to_next_mode = false;
                ROS_INFO("[Mode 07] Aim: Map change from 3rd to 1st floor");
                srvChangeMap.request.ns = "level_1";
                srvChangeMap.request.map_name = "localization";
                srvChangeMap.request.desired_frame_id = "summit_xl_map";
                srvChangeMap.request.desired_topic_name = "/summit_xl/map";
                srvChangeMap.request.desired_map_origin = poseMapOrigin[1];
                
                srv_map_change.call(srvChangeMap);
                if (srv_map_change.call(srvChangeMap) == true)
                {
                    is_cur_mode_completed = true;
                    ros::Duration(1.0).sleep(); // wait for the stable map change 
                    pubPoseWithCovStampedInitialpose.publish(poseWithCovStampedInitialpose);
                }
                else 
                {
                    ROS_INFO("[Mode 07](ERROR)requested, but not received True for map_change");
                    ROS_INFO_STREAM(srvChangeMap.response.msg);
                }                    
// map change needed
//                is_cur_mode_completed = true; // for test  

                mode_nexttime = MOVE_TO_ROBOHUB_FRONT_FROM_ELEV_INSIDE;

                break;
            case MOVE_TO_ROBOHUB_FRONT_FROM_ELEV_INSIDE: // SUMMIT operation
                flag_go_to_next_mode = false; // reset the flag after the action
                if (completion_ind_step_wam.data == 3)
                {
                    completion_ind_step_summit.data = 4;
                    ROS_INFO("[Mode 08] Aim: Move to the front of RoboHUB");

                    srv_clear_costmap.call(srvCmdEmpty);
                    ros::Duration(0.5).sleep(); // wait for clearing costmap 
                    pubPoseStampedSummit.publish(poseStampedTargetNav[ROBOHUB_FRONT]);
                }
                else
                {
                    ROS_INFO("[Mode 08](ERROR) not received a proper completion task from WAM");
                }
                is_auto_operation_enabled = false; // to stop reapation of the operation
                is_auto_operation_enabled_old = false; // to stop reapation of the operation
                ROS_INFO("Auto.Operation is OFF");

                mode_nexttime = WAIT;
                break;

        }
      }
    }
  }


///////////////////////////////////         Declarations      /////////////////////////////////////////////////////////////////

private:

  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pubArrivalStatusSummit;
  ros::Publisher pubArrivalStatusWAM;
  std_msgs::Bool rosBoolTrue;

  ros::Publisher pubPoseStampedSummit;
  ros::Publisher pubPoseStampedWAM;
  ros::Publisher pubPoseWithCovStampedInitialpose;

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

  // Service Client for multimap_server
  ros::ServiceClient srv_map_change;
  // Service Clients (for requests(i.eservice call) to WAM)
  ros::ServiceClient srv_cart_move, srv_ortn_move, srv_pose_move, srv_joint_move, srv_go_home; 
  ros::ServiceClient srv_hand_close_grsp, srv_hand_open_grsp;
  ros::ServiceClient srv_hand_close_sprd, srv_hand_open_sprd;   
  
  // Service Client for navigation (clear costmap)
  ros::ServiceClient srv_clear_costmap;

  // Service Clients for Inv.Kinematics of WAM
  ros::ServiceClient srv_ik_wam;

  // Service Clients for Knob Operation of WAM
  ros::ServiceClient srv_knob_operation_wam;

  // msgs for target poses
  geometry_msgs::PoseStamped poseStampedTargetNav[NUM_OF_NAV_TARGETS]; // Corridor Door, Elevator front, Elevator inside, RoboHub front

  // msgs map origins
  geometry_msgs::Pose poseMapOrigin[NUM_OF_MAPS]; // Corridor Door, Elevator front, Elevator inside, RoboHub front
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
  geometry_msgs::PoseWithCovarianceStamped poseWithCovStampedInitialpose; //to publish initial pose of amcl as needed

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
  geometry_msgs::Pose poseForGraspKnob;

 // for multimap_server
  multimap_server_msgs::PubMap srvChangeMap;
 // for wam
  uwarl_tasks::CartPosMove  srvCartMove;
  uwarl_tasks::OrtnMove  srvOrtnMove;
  uwarl_tasks::PoseMove  srvPoseMove;
  uwarl_tasks::JointMove  srvJointMove;

  // ik_solver of the wam
  ik_solver_barrett_wam::InvKineWam srvInvKineWam;
  tf2::Quaternion Bhand_grasp_ortn;
  geometry_msgs::Quaternion quatBHandGraspObjOrtn;

  // knob_operation of the wam
  uwarl_tasks::KnobOperation srvKnobOperationWam;
  
  std_srvs::Empty srvCmdEmpty;

  std::vector<double> read_pose_position;
  std::vector<double> read_pose_orientation;
  std::vector<double> read_values;

  geometry_msgs::PoseStamped poseStampedWAMpose;        // pose of the end-effector of WAM received from WAM
  sensor_msgs::JointState jointStateWAMjntState;

  bool is_robot_reached_target[2] = {true, true};
  std_msgs::Int8 completion_ind_step_wam;
  std_msgs::Int8 completion_ind_step_summit; 
  bool is_wam_moving = false;
  double cur_target_jnt[7];
  bool is_wam_reached = false;

  bool is_cur_mode_completed = true;
  bool flag_go_to_next_mode = false;

  int  mode_thistime; 
  int  mode_nexttime; 
  bool is_auto_operation_enabled = false;
  bool is_auto_operation_enabled_old = false;
  bool is_auto_operation_reset = true;

  int count_interval_fine_goal = 0;

  float des_pos_x,des_pos_y,des_pos_z;
  float des_ornt_x,des_ornt_y,des_ornt_z,des_ornt_w;

  float jntcmd[7];
  float jntact[7];

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
