#ifndef TRAJ_SAMP_H
#define TRAJ_SAMP_H

#include<mav_trajectory_generation/polynomial.h>
#include<mav_trajectory_generation/trajectory_sampling.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>

#include<mav_msgs/conversions.h>
#include<mav_msgs/default_topics.h>
#include<mav_msgs/eigen_mav_msgs.h>

#include<mav_planning_msgs/PolynomialSegment.h>
#include<mav_planning_msgs/PolynomialTrajectory.h>
#include<mav_planning_msgs/PolynomialSegment4D.h>
#include<mav_planning_msgs/PolynomialTrajectory4D.h>
#include<mav_planning_msgs/eigen_planning_msgs.h>
#include<mav_planning_msgs/conversions.h>

#include<mavros_msgs/State.h>

#include<ros/ros.h>
#include<std_srvs/Empty.h>
#include<std_msgs/Float32.h>
#include<tf/transform_datatypes.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>
// #include"traj_gen/FlatTarget.h"

double dt_ = 0.01;
mav_trajectory_generation::Trajectory trajectory_;
mavros_msgs::State current_state_;

void segmentsCallback(const mav_planning_msgs::PolynomialTrajectory::ConstPtr& msg);
void segments4DCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr& msg);
void stateCallback(const mavros_msgs::State::ConstPtr& msg);
void processTrajectory();

#endif