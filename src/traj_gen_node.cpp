#include"traj_gen/traj_gen.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_gen_node");
	ros::NodeHandle nh;

	const bool oneshot = false;
	const bool autostart = false;
	std::vector<double> target_pos;
	std::vector<double> middle_pos;
	std::vector<double> target_vel;

	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
	
	// flat_ref_pub_ = nh.advertise<traj_gen::FlatTarget>("reference/flatsetpoint", 1);
	flat_ref_pub_ = nh.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 1);
	yaw_ref_pub_ = nh.advertise<std_msgs::Float32>("reference/yaw", 1);

	ros::Publisher command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odomCallback);
	ros::Subscriber ref_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/reference/pose", 1, refPoseCallback);
	
	publish_timer_ = nh.createTimer(ros::Duration(dt_), &cmdTimerCallback, oneshot, autostart);

	// ros::Duration(0.5).sleep();
	nh.param<int>("/traj_gen_node/dimension", dimension_, 3);
	nh.param<bool>("/traj_gen_node/nonlinear_opt", nonlinear_, nonlinear_);
	nh.param<bool>("/traj_gen_node/input_middle", input_middle_, input_middle_);
	nh.param<double>("/traj_gen_node/max_v", max_v_, 2.0);
	nh.param<double>("/traj_gen_node/max_a", max_a_, 2.0);
	nh.param<double>("/traj_gen_node/max_ang_v", max_ang_v_, 1.0);
	nh.param<double>("/traj_gen_node/max_ang_a", max_ang_a_, 1.0);
	nh.getParam("/traj_gen_node/target_pos", target_pos);
	nh.getParam("/traj_gen_node/middle_pos", middle_pos);
	nh.getParam("/traj_gen_node/target_vel", target_vel);

	target_vel_ << target_vel[0], target_vel[1], target_vel[2];
	target_pos_ << target_pos[0], target_pos[1], target_pos[2];

	while(ros::ok() && !odom_received_)
	{
        ros::spinOnce();
    }

	std::printf("\n[ generator] Initialized\n");
	std::printf("[ generator] max_v = %.1f m/s, max_a = %.1f m/s^2\n", max_v_, max_a_);

	mav_trajectory_generation::Vertex::Vector vertices;
	mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_); 
	std::vector<double> segment_times;

	mav_trajectory_generation::PolynomialOptimization<N_> opt(dimension_);
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N_> non_opt(dimension_, parameters_);
	mav_trajectory_generation::Segment::Vector segments;

	visualization_msgs::MarkerArray markers;
    double distance = 0.5;

	mav_msgs::EigenTrajectoryPoint::Vector trajectory_point;

	if(dimension_ == 3)
	{
		start_pos_ = current_pose_.translation();
		start.makeStartOrEnd(start_pos_, derivative_to_optimize_);
		start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity_);
		vertices.push_back(start);

		geometry_msgs::Point mid_pt;
		mid_pt.x = (start_pos_[0] + target_pos_[0])/2;
		mid_pt.y = (start_pos_[1] + target_pos_[1])/2;
		mid_pt.z = (start_pos_[2] + target_pos_[2])/2;
		if(input_middle_)
		{
			middle_pos_ << middle_pos[0], middle_pos[1], middle_pos[2];
		}
		else
		{
			middle_pos_ << mid_pt.x, mid_pt.y, mid_pt.z;
		}
		
		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, middle_pos_);
		vertices.push_back(middle);

		end.makeStartOrEnd(target_pos_, derivative_to_optimize_);
		end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, target_vel_);
		vertices.push_back(end);
	}
	else if(dimension_ == 4)
	{
		start_pos_ = current_pose_.translation();
		start.makeStartOrEnd(Eigen::Vector4d(start_pos_[0], start_pos_[1], start_pos_[2], yaw_), derivative_to_optimize_);
		start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(current_velocity_[0], current_velocity_[1], current_velocity_[2], current_velocity_[2]));
		vertices.push_back(start);

		geometry_msgs::Point mid_pt;
		mid_pt.x = (start_pos_[0] + target_pos_[0])/2;
		mid_pt.y = (start_pos_[1] + target_pos_[1])/2;
		mid_pt.z = (start_pos_[2] + target_pos_[2])/2;
		if(input_middle_)
		{
			middle_pos_ << middle_pos[0], middle_pos[1], middle_pos[2];
		}
		else
		{
			middle_pos_ << mid_pt.x, mid_pt.y, mid_pt.z;
		}

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(middle_pos_[0], middle_pos_[1], middle_pos_[2], yaw_));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector4d(target_pos_[0], target_pos_[1], target_pos_[2], yaw_), derivative_to_optimize_);
		end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d(target_vel_[0], target_vel_[1], target_vel_[2], target_vel_[2]));
		vertices.push_back(end);
	}

	if(vertices.size() == 1)
	{
		ROS_ERROR("Can't generate trajectory with only one point!\n");
		ros::shutdown();
	}
	else
	{
		ROS_INFO_STREAM_ONCE("\n" << vertices);
	}

	segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

	if(!nonlinear_)
	{
		//linear
		std::printf("[ generator] Used linear optimization\n");
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
		opt.solveLinear();
		opt.getSegments(&segments);	
		opt.getTrajectory(&trajectory_);
	}
	else
	{
		//nonlinear
		std::printf("[ generator] Used nonlinear optimization\n");
		non_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
		non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
		non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
		non_opt.optimize();
		non_opt.getPolynomialOptimizationRef().getSegments(&segments);
		non_opt.getTrajectory(&trajectory_);
	}

	mav_trajectory_generation::drawMavTrajectory(trajectory_, distance, "map", &markers);
	marker_pub.publish(markers);
	
	publish_timer_.start();
	current_sample_time_ = 0.0;
	start_time_ = ros::Time::now();
	
	ros::spin();

	return 0;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry odom;
	odom = *msg;
	odom_received_ = true;
	tf::poseMsgToEigen(odom.pose.pose, current_pose_);
	tf::vectorMsgToEigen(odom.twist.twist.linear, current_velocity_);
	geometry_msgs::Quaternion q = odom.pose.pose.orientation;
	yaw_ = tf::getYaw(q);
}

void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::PoseStamped pose;
	pose = *msg;
}

void cmdTimerCallback(const ros::TimerEvent&)
{
	if(current_sample_time_ <= trajectory_.getMaxTime())
	{
		trajectory_msgs::MultiDOFJointTrajectory msg;
		mav_msgs::EigenTrajectoryPoint trajectory_point;
		bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory_, current_sample_time_, &trajectory_point);
		if(!success)
		{
			publish_timer_.stop();
		}
		mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
		msg.points[0].time_from_start = ros::Duration(current_sample_time_);
		msg.header.stamp = ros::Time::now();
		current_sample_time_ += dt_;

		// traj_gen::FlatTarget traj_msg;
		controller_msgs::FlatTarget traj_msg;
		traj_msg.type_mask = 2;
		traj_msg.header = msg.header;
		traj_msg.position = msg.points[0].transforms[0].translation;
		traj_msg.velocity = msg.points[0].velocities[0].linear;
		traj_msg.acceleration = msg.points[0].accelerations[0].linear;
		geometry_msgs::Quaternion quat = msg.points[0].transforms[0].rotation;
		double yaw = tf::getYaw(quat);
		std_msgs::Float32 yaw_msg;
		yaw_msg.data = static_cast<float>(yaw);

		flat_ref_pub_.publish(traj_msg);
		yaw_ref_pub_.publish(yaw_msg);
	}
	else
	{
		publish_timer_.stop();
		std::printf("\n\n\n[ INFO] Publish trajectory done\n");
		std::printf("  If want process other trajectory\n");
		std::printf("  Change param(s) target_pos (and middle_pos) in traj_gen.launch\n");
		std::printf("  And relaunch: roslaunch traj_gen traj_gen.launch [input_middle:=true]\n\n\n");
		ros::shutdown();
	}
}