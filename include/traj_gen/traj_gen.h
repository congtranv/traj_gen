#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include<mav_trajectory_generation/polynomial_optimization_linear.h>
#include<mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation_ros/trajectory_sampler_node.h>
#include<mav_trajectory_generation_ros/ros_visualization.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h> 
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/Point.h>
#include<nav_msgs/Odometry.h>
#include<mavros_msgs/State.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
#include<eigen_conversions/eigen_msg.h>

double max_v_;
double max_a_;
double max_ang_v_;
double max_ang_a_;
double yaw_;
double init_z_;

mavros_msgs::State current_state_;

Eigen::Vector3d target_pos_;
Eigen::Vector3d init_pos_;
Eigen::Vector3d target_vel_;

geometry_msgs::Point cmd_point_;
geometry_msgs::PoseStamped ref_pose_;
geometry_msgs::TwistStamped local_vel_;

Eigen::Affine3d current_pose_ = Eigen::Affine3d::Identity();
Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
Eigen::Vector3d current_ang_vel_ = Eigen::Vector3d::Zero();

int dimension_;
const int N_ = 10;
bool nonlinear_ = true;
const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

void cmdPointCallback(const geometry_msgs::Point::ConstPtr& msg);
void localVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void stateCallback(const mavros_msgs::State::ConstPtr& msg);

#endif