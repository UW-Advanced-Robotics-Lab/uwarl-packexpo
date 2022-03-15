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

void fnPubPose()
{

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
