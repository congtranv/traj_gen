#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<mavros_msgs/State.h>

mavros_msgs::State state_;
geometry_msgs::PoseStamped pose_;

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	state_ = *msg;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pose_ = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "reference_pose_node");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);
	ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallback);

	ros::Publisher ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/reference/pose", 10);

	while(ros::ok() && !state_.connected)
	{
		ros::spinOnce();
	}
	for(int i=0; i<20; i++)
	{
		ros::spinOnce();
	}
	while(ros::ok())
	{
		ROS_INFO_ONCE("ref_pose_node: publishing");
		geometry_msgs::PoseStamped msg;
		msg = pose_;
		ref_pose_pub.publish(msg);
		ros::spinOnce();
	}
	return 0;
}