#include"traj_gen/traj_gen.h"

// mav_trajectory_generation::Vertex::Vector vertices;
// mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_); 
// std::vector<double> segment_times;

// mav_trajectory_generation::PolynomialOptimization<N_> opt(dimension_);
// mav_trajectory_generation::PolynomialOptimizationNonLinear<N_> non_opt(dimension_, parameters_);
// mav_trajectory_generation::Segment::Vector segments;
// mav_trajectory_generation::Trajectory trajectory;

// visualization_msgs::MarkerArray markers;
// double distance = 0.5;

// ros::Publisher traj_pub;
// ros::Publisher traj4d_pub;
// ros::Publisher marker_pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_gen_node");
	ros::NodeHandle nh;

	// ros::Publisher traj_pub  = nh.advertise<mav_planning_msgs::PolynomialTrajectory>("path_segments", 1);
	// ros::Publisher traj4d_pub = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments_4D", 1);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
	// traj_pub  = nh.advertise<mav_planning_msgs::PolynomialTrajectory>("path_segments", 1);
	// traj4d_pub = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("path_segments_4D", 1);
	// marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);

	ros::Publisher command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
	ros::Subscriber ref_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("reference/pose", 1, refPoseCallback);
	ros::Subscriber cmd_point_sub = nh.subscribe<geometry_msgs::Point>("/command/point", 1, cmdPointCallback);
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_body", 1, localVelCallback);
	
	// ros::Duration(0.5).sleep();
	nh.param<int>("/traj_gen_node/dimension", dimension_, 3);
	nh.param<bool>("/traj_gen_node/nonlinear_opt", nonlinear_, nonlinear_);
	nh.param<double>("/traj_gen_node/max_v", max_v_, 2.0);
	nh.param<double>("/traj_gen_node/max_a", max_a_, 2.0);
	nh.param<double>("/traj_gen_node/max_ang_v", max_ang_v_, 1.0);
	nh.param<double>("/traj_gen_node/max_ang_a", max_ang_a_, 1.0);
	nh.param<double>("/traj_gen_node/init_z", init_z_, 1.5);
	
	target_vel_ << 0.0, 0.0, 0.0;
	init_pos_ << 0.0, 0.0, init_z_;
	// ref_pose_.pose.position.x = ref_pose_.pose.position.y = ref_pose_.pose.position.z = 0.0;
	// cmd_point_.x = cmd_point_.y = cmd_point_.z = 5.0;
	// yaw_ = 0.0;

	while(ros::ok() && !current_state_.connected)
	{
        // ROS_INFO_ONCE("\nWaiting for FCU connection\n");
        ros::spinOnce();
    }
    // ROS_INFO("FCU connected \n");

	for(int i=0;i<100;i++)
	{
		ros::spinOnce();
	}

	// tf::poseMsgToEigen(ref_pose_.pose, current_pose_);
	// geometry_msgs::Quaternion q = ref_pose_.pose.orientation;
	// yaw_ = tf::getYaw(q);
	ROS_INFO_STREAM("generator: yaw\n" << yaw_);
	ROS_INFO("generator: Initialized");

	mav_trajectory_generation::Vertex::Vector vertices;
	mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_); 
	std::vector<double> segment_times;

	mav_trajectory_generation::PolynomialOptimization<N_> opt(dimension_);
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N_> non_opt(dimension_, parameters_);
	mav_trajectory_generation::Segment::Vector segments;
	mav_trajectory_generation::Trajectory trajectory;

	visualization_msgs::MarkerArray markers;
    double distance = 0.5;

	mav_msgs::EigenTrajectoryPoint::Vector trajectory_point;
	double dt = 0.01;

	// while(ros::ok())
	// { 
		// tf::poseMsgToEigen(ref_pose_.pose, current_pose_);
		// geometry_msgs::Quaternion q = ref_pose_.pose.orientation;
		// yaw_ = tf::getYaw(q);
		// tf::vectorMsgToEigen(local_vel_.twist.linear, current_velocity_);
		// tf::vectorMsgToEigen(local_vel_.twist.angular, current_ang_vel_);
		// tf::pointMsgToEigen(cmd_point_, target_pos_);
		
		if(dimension_ == 3)
		{
			// start.makeStartOrEnd(Eigen::Vector3d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z), derivative_to_optimize_);
			start.makeStartOrEnd(init_pos_, derivative_to_optimize_);
			start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);
			vertices.push_back(start);

			geometry_msgs::Point mid_pt;
			mid_pt.x = (ref_pose_.pose.position.x + cmd_point_.x)/2;
			mid_pt.y = (ref_pose_.pose.position.y + cmd_point_.y)/2;
			mid_pt.z = (ref_pose_.pose.position.z + cmd_point_.z)/2;

			middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(mid_pt.x, mid_pt.y, mid_pt.z));
			vertices.push_back(middle);

			end.makeStartOrEnd(Eigen::Vector3d(cmd_point_.x, cmd_point_.y, cmd_point_.z), derivative_to_optimize_);
			end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, target_vel_);
			vertices.push_back(end);
		}
		else if(dimension_ == 4)
		{
			// start.makeStartOrEnd(Eigen::Vector4d(ref_pose_.pose.position.x, ref_pose_.pose.position.y, ref_pose_.pose.position.z, yaw_), derivative_to_optimize_);
			start.makeStartOrEnd(Eigen::Vector4d(init_pos_[0], init_pos_[1], init_pos_[2], yaw_), derivative_to_optimize_);
			start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(current_velocity_[0], current_velocity_[1], current_velocity_[2], current_velocity_[2]));
			vertices.push_back(start);

			geometry_msgs::Point mid_pt;
			mid_pt.x = (ref_pose_.pose.position.x + cmd_point_.x)/2;
			mid_pt.y = (ref_pose_.pose.position.y + cmd_point_.y)/2;
			mid_pt.z = (ref_pose_.pose.position.z + cmd_point_.z)/2;

			middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(mid_pt.x, mid_pt.y, mid_pt.z, yaw_));
			vertices.push_back(middle);

			end.makeStartOrEnd(Eigen::Vector4d(cmd_point_.x, cmd_point_.y, cmd_point_.z, yaw_), derivative_to_optimize_);
			end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(target_vel_[0], target_vel_[1], target_vel_[2], target_vel_[2]));
			vertices.push_back(end);
		}

		if(vertices.size() == 1)
		{
			ROS_ERROR("generator: Can't generate trajectory with only one point!");
			ros::shutdown();
		}
		else
		{
			ROS_INFO_STREAM_ONCE("vertices:\n" << vertices);
		}

		segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

		if(!nonlinear_)
		{
			//linear
			ROS_INFO_ONCE("generator: linear optimization");
			opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
			opt.solveLinear();
			opt.getSegments(&segments);	
			opt.getTrajectory(&trajectory);
		}
		else
		{
			//nonlinear
			ROS_INFO_ONCE("generator: nonlinear optimization");
			non_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
			non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
			non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
			non_opt.optimize();
			non_opt.getPolynomialOptimizationRef().getSegments(&segments);
			non_opt.getTrajectory(&trajectory);
		}

		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, "map", &markers);
		marker_pub.publish(markers);
		
		// if(dimension_ == 3)		
		// {
		// 	mav_planning_msgs::PolynomialTrajectory poly_msg;
		// 	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &poly_msg);
		// 	poly_msg.header.frame_id = "map";
		// 	traj_pub.publish(poly_msg);
		// 	ROS_INFO("generator: segment publishing");
		// }
		// else if(dimension_ == 4)
		// {
		// 	mav_planning_msgs::PolynomialTrajectory4D poly_msg;
		// 	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &poly_msg);
		// 	poly_msg.header.frame_id = "map";
		// 	traj4d_pub.publish(poly_msg);
		// 	ROS_INFO("generator: segment 4D publishing");
		// }
		
		bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, dt, &trajectory_point);
		if(success)
		{
			trajectory_msgs::MultiDOFJointTrajectory msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
			command_pub.publish(msg);
			ROS_INFO("sampler: publish trajectory");
		}
		else
		{
			ROS_ERROR("sampler: failed to sample trajectory");
		}
		

	// 	ros::spinOnce();
	// }	
	ros::spin();

	return 0;
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
	current_state_ = *msg;
}

void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ref_pose_ = *msg;
	tf::poseMsgToEigen(ref_pose_.pose, current_pose_);
	geometry_msgs::Quaternion q = ref_pose_.pose.orientation;
	yaw_ = tf::getYaw(q);
	// ROS_INFO_STREAM("generator: yaw\n" << yaw_);
	// ROS_INFO_STREAM_ONCE("generator: ref pose\n" << ref_pose_);
	// ROS_INFO("generator: pose callback");
}

void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel_ = *msg;
	tf::vectorMsgToEigen(local_vel_.twist.linear, current_velocity_);
	// tf::vectorMsgToEigen(local_vel_.twist.angular, current_ang_vel_);
	// ROS_INFO_STREAM_ONCE("generator: velocity\n" << local_vel_);
}

void cmdPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	cmd_point_ = *msg;
	// tf::pointMsgToEigen(cmd_point_, target_pos_);
	// ROS_INFO_STREAM_ONCE("generator: cmd point\n" << cmd_point_);
}