#include "ros/ros.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "uwarl_tasks/MsgUwarl_tasks.h"

//  Declaration of the required variables
geometry_msgs::Twist cmd_vel, cmd_vel_old;
geometry_msgs::Pose current_pose, target_pose;
double roll,pitch,yaw;
bool goal_reached;

std::string global_name, relative_name, default_param;

//void msgCallback(const uwarl_tasks::MsgUwarl_tasks::ConstPtr& msg)
void msgCallbackTarget(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	target_pose.position.x = msg->point.x;
	target_pose.position.y = msg->point.y;
	target_pose.orientation.z = current_pose.orientation.z;
	target_pose.orientation.w = current_pose.orientation.w;
	goal_reached = false;
}

// for use of Robohub
void msgCallbackCurrentPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	current_pose = msg->pose.pose;
	tf::Quaternion q(   msg->pose.pose.orientation.x,
					    msg->pose.pose.orientation.y,
					    msg->pose.pose.orientation.z,
					    msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);
}


void msgCallbackCurrentVel(const geometry_msgs::Twist::ConstPtr& msg)
{
	cmd_vel_old = *msg;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "UWARL_controller");
	serviceCore_packExpo();
	ros::spin();
}

// int main(int argc, char **argv)
// {
// 	float kp = 1.0; // p-gain for the controller
// 	float ki = 0.1; // I-gain for the controller
// 	float dx = 0.0;
// 	float dy = 0.0;
// 	float dz_th = 0.0;
// 	float ds = 0.0;	
// 	float max_vel_trans = 3.0;
// 	float max_acc_trans = 15.0;
// 	float phi = 0.0; // angle of line connected from the current to target position
// 	float vel_trans_old = 0.0;
// 	float vel_trans = 0.0;
	
// 	float goal_tol_distance = 0.05;
// 	float goal_tol_distance_rough = 0.1; 
// 	float goal_tol_angle = 0.1;
// 	bool precision_control = false;
// 	bool enable_precision_control = false;
// 	goal_reached = true;

// 	int freq = 5;
// 	double deltaT = 1/double(freq);

// 	geometry_msgs::Twist max_acc;
// 	geometry_msgs::Twist max_vel;

// 	ros::init(argc, argv, "UWARL_controller");

// 	ros::NodeHandle nh;
	
// 	// values initialization
// 	max_acc.linear.x = 1.5;
// 	max_acc.linear.y = 1.5;
// 	max_acc.angular.z = 1.5;

// 	max_vel.linear.x = 1.5;
// 	max_vel.linear.y = 1.5;
// 	max_vel.angular.z = 1.5;

// 	cmd_vel.linear.x = 0;
// 	cmd_vel.linear.y = 0;
// 	cmd_vel.linear.z = 0;
// 	cmd_vel.angular.x = 0;
// 	cmd_vel.angular.y = 0;
// 	cmd_vel.angular.z = 0;

// 	cmd_vel_old = cmd_vel;

// 	current_pose.position.x = 0;
// 	current_pose.position.y = 0;
// 	current_pose.position.z = 0;
// 	current_pose.orientation.x = 0;
// 	current_pose.orientation.y = 0;
// 	current_pose.orientation.z = 0;
// 	current_pose.orientation.w = 1;
// 	yaw = 0.0;

// 	target_pose = current_pose;

// //  acnouncement for subscribers 
// 	ros::Subscriber target_pose_subscriber = nh.subscribe("/clicked_point", 100, msgCallbackTarget);
// 	ros::Subscriber current_pose_subscriber = nh.subscribe("/summit_xl_a/amcl_pose", 100, msgCallbackCurrentPos);
// 	ros::Subscriber current_vel_subscriber = nh.subscribe("/summit_xl_a/robotnik_base_control/cmd_vel", 100, msgCallbackCurrentVel);

// 	ros::Subscriber target_pose_subscriber = nh.subscribe("/clicked_point", 100, msgCallbackTarget);

// //  acnouncement for publisher	
// 	ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/summit_xl_a/move_base/cmd_vel",10);
// 	ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/summit_xl_a/move_base/cmd_vel",10);
// 	ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/summit_xl_a/move_base/cmd_vel",10);

// //  loop_rate
//     ros::Rate loop_rate(freq);

// //	ros::spin();

//     if (enable_precision_control == false)
//     	{goal_reached = goal_tol_distance_rough;}

//    	while(ros::ok())
//    	{
// 		ros::spinOnce();

// 		dx = target_pose.position.x - current_pose.position.x;
// 		dy = target_pose.position.y - current_pose.position.y;
// 		dz_th = target_pose.orientation.z - current_pose.orientation.z;
// 		ds = sqrt(dx*dx+dy*dy);
// 		phi = atan2(dy,dx);

// // Check if the goal has been reached
// 		if (ds < goal_tol_distance_rough)
// 		{
// 			if (ds < goal_tol_distance)
// 			{
// 				// Anounce goal has been reached
// 				if (goal_reached == false)
// 				{
// 					ROS_INFO("goal_reached! Dist_to_Goal = %.2fmm",1000*ds);	
// 					if (enable_precision_control == true)
// 					{ROS_INFO("precision control off.");}												
// 				}
// 				goal_reached = true;	
// 				precision_control = false;			
// 			}
// 			else
// 			{	
// 				precision_control = true;
// 				if (enable_precision_control == true)
// 				{ROS_INFO("precision control on. Dist to goal= %.2fmm",1000*ds);}
// 			}
// 		}

// // Compute cmd_vel
// 		if (goal_reached == true)
// 		{		
// 			cmd_vel.linear.x = 0;
// 			cmd_vel.linear.y = 0;
//     		cmd_vel.angular.z = 0;		
//     	}
//     	else
//     	{
// 			//  Calculate cmd_vel, poisition controller, based on absolute position
// //			cmd_vel.linear.x = kp*(dx*cos(yaw) + dy*sin(yaw));
// //			cmd_vel.linear.y = kp*(-dx*sin(yaw) + dy*cos(yaw));
// 			//cmd_vel.angular.z = kp*(target_pose.orientation.z - current_pose.orientation.z)    

 
//  /*   		// Check the max_acc
//     		if (fabs(cmd_vel.linear.x - cmd_vel_old.linear.x) > max_acc.linear.x * deltaT)
//     			{
//     				if(cmd_vel.linear.x - cmd_vel_old.linear.x < 0)
//     				{cmd_vel.linear.x = cmd_vel_old.linear.x - max_acc.linear.x * deltaT;}
//     				else
//     				{cmd_vel.linear.x = cmd_vel_old.linear.x + max_acc.linear.x * deltaT;}    				
//     			}
//     		if (fabs(cmd_vel.linear.y - cmd_vel_old.linear.y) > max_acc.linear.y * deltaT)
//     			{
//     				if(cmd_vel.linear.y - cmd_vel_old.linear.y < 0)
//     				{cmd_vel.linear.y = cmd_vel_old.linear.y - max_acc.linear.y * deltaT;}
//     				else
//     				{cmd_vel.linear.y = cmd_vel_old.linear.y + max_acc.linear.y* deltaT;}   			
//     			}
//     		if (fabs(cmd_vel.angular.z - cmd_vel_old.angular.z) > max_acc.angular.z  * deltaT)
//     			{
//      				if(cmd_vel.angular.z - cmd_vel_old.angular.z < 0)
//     				{cmd_vel.angular.z = cmd_vel_old.angular.z - max_acc.angular.z * deltaT;}
//     				else
//     				{cmd_vel.angular.z = cmd_vel_old.angular.z + max_acc.angular.z * deltaT;} 
//     			}		

// 			// Check the max_vel
//     		if (cmd_vel.linear.x > max_vel.linear.x)
//     			{cmd_vel.linear.x = max_vel.linear.x;}
//     		else if (cmd_vel.linear.x < -max_vel.linear.x)
//     			{cmd_vel.linear.x = -max_vel.linear.x;}
//     		if (cmd_vel.linear.y > max_vel.linear.y)
//     			{cmd_vel.linear.y = max_vel.linear.y;}
//     		else if (cmd_vel.linear.y < -max_vel.linear.y)
//     			{cmd_vel.linear.y = -max_vel.linear.y;}
//     		if (cmd_vel.angular.z > max_vel.angular.z)     			{cmd_vel.angular.z = max_vel.angular.z;}
//     		else if (cmd_vel.angular.z < -max_vel.angular.z)    	{cmd_vel.angular.z = -max_vel.angular.z;}

// */
// // Second way of computing cmd_vel
//     		if (precision_control == true && enable_precision_control == true)
// 			{
// 				vel_trans = 0.2*kp*ds;
// 			}
// 			else
// 			{
// 	    		vel_trans = kp*ds;				
// 			}
//     		if (fabs(vel_trans - vel_trans_old) > max_acc_trans * deltaT)
//     		{
//     			if(vel_trans - vel_trans_old < 0)
//     				{vel_trans = vel_trans_old - max_acc_trans*deltaT;}
//     			else
// 	    			{vel_trans = vel_trans_old + max_acc_trans*deltaT;}
//     		}

//     		if (vel_trans > max_vel_trans)    { vel_trans = max_vel_trans;}
//     		else if (vel_trans < -max_vel_trans) { vel_trans = -max_vel_trans;}

// 			cmd_vel.linear.x = vel_trans*cos(phi-yaw);
// 			cmd_vel.linear.y = vel_trans*sin(phi-yaw);

//     	}	
	
// // Publish cmd_vel
// 		cmd_vel_publisher.publish(cmd_vel);

//    		loop_rate.sleep();   		
//    	}

//     return 0;
// }

serviceCore_packExpo()
{
	fnInitParam();

	pubServiceStatusSummit = nh_.advertise<uwarl_tasks::ServiceStatus>("/summit_xl_a/service_status",1);
	pubPlaySoundSummit = nh_.advertise<std_msgs::String>("/summit_xl_a/play_sound_file",1);
	pubPoseStampedSummit = nh_.advertise<geometry_msgs::PoseStamped>("/summit_xl_a/move_base_simple/goal",1);

	pubTaskOrderSummit = nh_.subscribe("/summit_xl_a/task_order", 1, &serviceCore_packExpo::cbReceiveTaskOrder, this);

	subArrivalStatusSummit = nh_.subscribe("/summit_xl_a/move_base/result", 1, &serviceCore_packExpo::cbCheckArrivalStatusSummit, this);

	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		fnPubServiceStatus();

		fnPubPose();
		ros:spinOnce();
		loop_rate.sleep();
	}
}

void fnInitParam()
{
	nh_.getParam("pick_table_pose_summit/position", target_pose_position);
	nh_.getParam("pick_table_pose_summit/orientation", target_pose_orientation);

	PoseStampedPickTable[0].header.frame_id = "map";
	PoseStampedPickTable[0].header.stamp = ros::Time::now();

	PoseStampedPickTable[0].pose.position.x = target_pose_position[0];
	PoseStampedPickTable[0].pose.position.y = target_pose_position[1];
	PoseStampedPickTable[0].pose.position.z = target_pose_position[2];

	PoseStampedPickTable[0].pose.orientation.x = target_pose_orientation[0];
	PoseStampedPickTable[0].pose.orientation.y = target_pose_orientation[1];
	PoseStampedPickTable[0].pose.orientation.z = target_pose_orientation[2];
	PoseStampedPickTable[0].pose.orientation.w = target_pose_orientation[3];

	nh_.getParam("place_table_pose_summit/position", target_pose_position);
	nh_.getParam("place_table_pose_summit/orientation", target_pose_orientation);

	PoseStampedPlaceTable[0].header.frame_id = "map";
	PoseStampedPlaceTable[0].header.stamp = ros::Time::now();

	PoseStampedPlaceTable[0].pose.position.x = target_pose_position[0];
	PoseStampedPlaceTable[0].pose.position.y = target_pose_position[1];
	PoseStampedPlaceTable[0].pose.position.z = target_pose_position[2];

	PoseStampedPlaceTable[0].pose.orientation.x = target_pose_orientation[0];
	PoseStampedPlaceTable[0].pose.orientation.y = target_pose_orientation[1];
	PoseStampedPlaceTable[0].pose.orientation.z = target_pose_orientation[2];
	PoseStampedPlaceTable[0].pose.orientation.w = target_pose_orientation[3];

}

void fnPubPose()
{
	if (is_robot_reached_target[ROBOT_NUMBER])
	{
		if (robot_service_sequence[ROBOT_NUMBER] == 1)
		{
			// fnPublishVoiceFilePath(ROBOT_NUMBER, "~/voice/voice1-2.mp3");
			ROS_INFO("service sequence = 1");
			robot_service_sequence[ROBOT_NUMBER] = 2;
		}
		else if (robot_service_sequence[ROBOT_NUMBER] == 2)
		{
			pubPoseStampedSummit.publish(PoseStampedPlaceTable[item_num_chosen_by_pad[ROBOT_NUMBER]]);
			is_robot_reached_target[ROBOT_NUMBER] = false;
			robot_service_sequence[ROBOT_NUMBER] = 3;
		}
		else if (robot_service_sequence[ROBOT_NUMBER] == 3)
		{
			// fnPublishVoiceFilePath(ROBOT_NUMBER, "~/voice/voice1-3.mp3");
			ROS_INFO("service sequence = 3");
			robot_service_sequence[ROBOT_NUMBER] = 4;
		}
		else if (robot_service_sequence[ROBOT_NUMBER] == 4)
		{
			pubPoseStampedSummit.publish(PoseStampedPickTable[ROBOT_NUMBER]);
			is_robot_reached_target[ROBOT_NUMBER] = false;
			robot_service_sequence[ROBOT_NUMBER] = 5;
		}
		else if (robot_service_sequence[ROBOT_NUMBER] == 5)
		{
			// fnPublishVoiceFilePath(ROBOT_NUMBER, "~/voice/voice1-4.mp3");
			ROS_INFO("service sequence = 5");
			robot_service_sequence[ROBOT_NUMBER] = 0;
			is_item_available[item_num_chosen_by_pad[ROBOT_NUMBER]] =1;
			item_num_chosen_by_pad[ROBOT_NUMBER] = -1;
		}
	}

}

void cbReceiveTaskOrder(const uwarl_tasks::TaskOrder taskOrder)
{
	int pad_number = taskOrder.task_number;
	int item_number = taskOrder.item_number;

	if (is_item_available[item_number] != 1)
	{
		ROS_INFO("Chosen item is currently unavailable");
		return;	
	}

	if (robot_service_sequence[task_number] != 0)
	{
		ROS_INFO("Your Summit is currently on servicing");
		return;
	}

	if (item_num_chosen_by_pad[task_number] != 1)
	{
		ROS_INFO("Your Summit is currently on servicing");
		return;
	}

	item_num_chosen_by_pad[task_number] = item_number;
	robot_service_sequence[pad_number] = 1; // just left from the picking table
	is_item_available[item_number] = 0;
}

void cbCheckArrivalStatusSummit(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
{
	if(rcvMoveBaseActionResult.status.status == 3)
	{
		is_robot_reached_target[ROBOT_NUMBER_SUMMIT] = true;
	}
	else
	{
		// funtions are inserted when the robot meets obstacle / cannot find the path, etc...
	}

}
