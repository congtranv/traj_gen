#include"traj_gen/traj_samp.h"

ros::Publisher command_pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_samp_node");
	ros::NodeHandle nh;

	command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
	ros::Publisher flat_ref_pub = nh.advertise<traj_gen::FlatTarget>("reference/flatsetpoint", 1);
	ros::Publisher yaw_ref_pub = nh.advertise<std_msgs::Float32>("reference/yaw", 1);

	ros::Subscriber trajectory_sub = nh.subscribe<mav_planning_msgs::PolynomialTrajectory>("path_segments", 10, segmentsCallback);
	ros::Subscriber trajectory_4D_sub = nh.subscribe<mav_planning_msgs::PolynomialTrajectory4D>("path_segments_4D", 10, segments4DCallback);
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, stateCallback);

	// ros::Duration(0.5).sleep();
	nh.param<double>("/traj_samp_node/dt", dt_, dt_);

	// while(ros::ok() && !current_state_.connected)
	// {
        // ROS_INFO_ONCE("\nWaiting for FCU connection\n");
        // ros::spinOnce();
    // }
    // ROS_INFO("FCU connected \n");
	for(int i=0;i<100;i++)
	{
		ros::spinOnce();
	}
	ROS_INFO("sampler: Initialized");

	ros::spin();

	return 0;
}

void segmentsCallback(const mav_planning_msgs::PolynomialTrajectory::ConstPtr& msg)
{
	mav_planning_msgs::PolynomialTrajectory segment;
	segment = *msg;
	ROS_INFO("sampler: segment callback");
	if(segment.segments.empty())
	{
		ROS_WARN("sampler: received empty waypoint message");
	}
	else
	{
		ROS_INFO("sampler: received %lu new waypoints", segment.segments.size());
	}
	bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(segment, &trajectory_);
	if(!success)
	{
		ROS_WARN("sampler: failed to transform polynomial trajectory msg to trajectory");
	}
	processTrajectory();
}

void segments4DCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr& msg)
{
	mav_planning_msgs::PolynomialTrajectory4D segment;
	segment = *msg;
	ROS_INFO("sampler: segment 4D callback");
	if(segment.segments.empty())
	{
		ROS_WARN("sampler: received empty waypoint message");
	}
	else
	{
		ROS_INFO("sampler: received %lu new waypoints", segment.segments.size());
	}
	bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(segment, &trajectory_);
	if(!success)
	{
		ROS_WARN("sampler: failed to transform polynomial trajectory msg to trajectory");
	}
	processTrajectory();
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state_ = *msg;
	ROS_INFO("sampler: state callback");
}

void processTrajectory()
{
	mav_msgs::EigenTrajectoryPoint::Vector trajectory_point;
	trajectory_msgs::MultiDOFJointTrajectory msg;
	
	mav_trajectory_generation::sampleWholeTrajectory(trajectory_, dt_, &trajectory_point);

	mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
	command_pub.publish(msg);
	ROS_INFO("sampler: processing trajectory");
}