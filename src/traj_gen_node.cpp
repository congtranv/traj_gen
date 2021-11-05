#include<mav_trajectory_generation/polynomial_optimization_linear.h>
#include<mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation_ros/trajectory_sampler_node.h>
#include<mav_trajectory_generation_ros/ros_visualization.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h> 
#include<visualization_msgs/MarkerArray.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<eigen_conversions/eigen_msg.h>

double v_max = 2.0;
double a_max = 2.0;

Eigen::Vector3d pos_target, vel_target;
double pos_x = 5.0, pos_y = 5.0, pos_z = 3.0;
double vel_x = 0.0, vel_y = 0.0, vel_z = 0.0;
nav_msgs::Odometry current_odom;
Eigen::Vector3d current_velocity = Eigen::Vector3d::Zero();
Eigen::Affine3d current_pose = Eigen::Affine3d::Identity();

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	current_odom = *odom;
	tf::poseMsgToEigen(current_odom.pose.pose, current_pose);
	tf::vectorMsgToEigen(current_odom.twist.twist.linear, current_velocity);
	// ROS_INFO_STREAM("Current position:\n" << current_pose.translation() << "\n");
	// ROS_INFO_STREAM("Current velocity:\n" << current_velocity << "\n");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "traj_gen_node");
	ros::NodeHandle nh;

	ros::Duration(0.5).sleep();
	nh.param<double>("/traj_gen_node/max_v", v_max, v_max);
	nh.param<double>("/traj_gen_node/max_a", a_max, a_max);
	nh.param<double>("/traj_gen_node/target_pos_x", pos_x, pos_x);
	nh.param<double>("/traj_gen_node/target_pos_y", pos_y, pos_y);
	nh.param<double>("/traj_gen_node/target_pos_z", pos_z, pos_z);
	nh.param<double>("/traj_gen_node/target_vel_x", vel_x, vel_x);
	nh.param<double>("/traj_gen_node/target_vel_y", vel_y, vel_y);
	nh.param<double>("/traj_gen_node/target_vel_z", vel_z, vel_z);

	ros::Publisher traj_pub  = nh.advertise<mav_planning_msgs::PolynomialTrajectory>("path_segments", 0);
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);

	ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, odomCallback);

	for(int i=0;i<10;i++)
	{
		ros::spinOnce();
	}

	pos_target << pos_x, pos_y, pos_z;
	vel_target << vel_x, vel_y, vel_z;

	const int dimension = 3;
	mav_trajectory_generation::Vertex::Vector vertices;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
	std::vector<double> segment_times;
	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
	mav_trajectory_generation::Segment::Vector segments;
	mav_trajectory_generation::Trajectory trajectory;

	mav_trajectory_generation::NonlinearOptimizationParameters parameters;
	parameters.max_iterations = 1000;
	parameters.f_rel = 0.05;
	parameters.x_rel = 0.1;
	parameters.time_penalty = 500.0;
	parameters.initial_stepsize_rel = 0.1;
	parameters.inequality_constraint_tolerance = 0.1;
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N> non_opt(dimension, parameters);

	visualization_msgs::MarkerArray markers;
	double distance = 1.0;
	std::string frame_id = "map";
	mav_planning_msgs::PolynomialTrajectory poly_msg;


	start.makeStartOrEnd(current_pose.translation(), derivative_to_optimize);
	start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, current_velocity);
	vertices.push_back(start);

	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6,7,5));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6,-7,5));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-6,-7,5));
	vertices.push_back(middle);

	end.makeStartOrEnd(pos_target, derivative_to_optimize);
	end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_target);
	vertices.push_back(end);

	segment_times = estimateSegmentTimes(vertices, v_max, a_max);

	//linear
	// opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	// opt.solveLinear();
	// opt.getSegments(&segments);	
	// opt.getTrajectory(&trajectory);

	//nonlinear
	non_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
	non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
	non_opt.optimize();
	non_opt.getPolynomialOptimizationRef().getSegments(&segments);
	non_opt.getTrajectory(&trajectory);

	// double sampling_time = 2.0;
	// int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
	// Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);	
	// ROS_INFO_STREAM("Sample:\n" << sample << "\n");

	mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
	ROS_INFO_STREAM("Marker\n" << markers << "\n");
	// marker_pub.publish(markers);

		
	mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &poly_msg);
	poly_msg.header.frame_id = "map";
	// ROS_INFO_STREAM("Publish Polynomial Trajectory\n" << poly_msg << "\n");
	// traj_pub.publish(poly_msg);
	// ROS_INFO_STREAM("Finish Publish Polynomial Trajectory\n");

	// ros::spin();
	while(ros::ok())
	{
		traj_pub.publish(poly_msg);
		marker_pub.publish(markers);
		ros::spinOnce();
	}
	
	return 0;
}