#include<ros/ros.h>
#include<mav_trajectory_generation/trajectory_sampling.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<mav_msgs/conversions.h>
#include<mav_planning_msgs/PolynomialSegment.h>
#include<mav_planning_msgs/PolynomialTrajectory.h>
#include<mav_planning_msgs/eigen_planning_msgs.h>
#include<mav_planning_msgs/conversions.h>

double dt = 0.01;
mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
mav_trajectory_generation::Trajectory trajectory;
trajectory_msgs::MultiDOFJointTrajectory traj_msg;

void segmentCallback(const mav_planning_msgs::PolynomialTrajectory::ConstPtr& segment_msg)
{
	mav_planning_msgs::PolynomialTrajectory msg;
	msg = *segment_msg;
	if(msg.segments.empty())
	{
		ROS_WARN("Empty segment message");
	}
	else
	{
		ROS_INFO_ONCE("Received %lu waypoints", msg.segments.size());
	}                                         
	bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(msg, &trajectory);
	if (!success)
	{
		return;
	}
	mav_trajectory_generation::sampleWholeTrajectory(trajectory, dt, &trajectory_points);
	mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_points, &traj_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_samp_node");
	ros::NodeHandle nh;

	ros::Duration(0.5).sleep();
	nh.param<double>("/traj_samp_node/dt", dt, dt);

	ros::Publisher command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 0);

	ros::Subscriber segment_sub = nh.subscribe<mav_planning_msgs::PolynomialTrajectory>("path_segments", 1, segmentCallback);

	for(int i=0;i<10;i++)
	{
		ros::spinOnce();
	}

	while(ros::ok())
	{
		traj_msg.header.frame_id = "map";
		command_pub.publish(traj_msg);
		ros::spinOnce();
	}

	return 0;
}