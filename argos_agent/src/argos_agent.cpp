/*
 * ArgosAgent Class - Master Project Timo Müller SS25
 * 
 */

#include "argos_agent/argos_agent.h"

#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <stdlib.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <freicar_msgs/ControlCommand>

#include <cmath>
#include <csignal>

sig_atomic_t volatile g_request_shutdown = 0;

ArgosAgent::ArgosAgent()
  : nh_private_("~"), tf_listener_(tf_buffer_), control_mode_(false) {
  // Receive parameters from parameter server
  nh_private_.getParam("agent_name", agent_name_);
  nh_private_.getParam("global_frame", global_frame_);
  nh_private_.getParam("p", p_);
  nh_private_.getParam("d", d_);
  nh_private_.getParam("i", i_);
  nh_private_.getParam("desired_velocity", desired_velocity_);
  nh_private_.getParam("lookahead_distance" lookahead_distance_);
  
  // TF parameters
  std::string rear_axis_frame_ = agent_name_ + "/rear_axis";
  std::string base_link_frame_ = agent_name_ + "/base_link";

  // Member variables
  geometry_msgs::Pose lookahead_point_;
  nav_msgs::Odometry odometry_;

  // PID
  ros::Duration prev_time_ = ros::Time::now();
  double prev_error_ = 0.0;
  double integral = 0.0;
  double throttle_ = 0.0;

  // Pure Pursuit
  double theta_ = 0.0;

  // Set up subscribers and publishers
  // Pubs
  control_pub_ = nh_.advertise<freicar_msgs::ControlCommand>("/" + agent_name_ + "/control", 1000);
  control_mode_pub_ = nh_.advertise<std::msgs::Bool>("/" + agent_name_ + "/control_mode", 1000);
  // Subs
  lookahead_point_sub_ = nh_.subscribe("/" + agent_name_ + "/lookahead_point", 1000, &ArgosAgent::LookaheadCallback(), this);
  odometry_sub_ = nh_.subscribe("/" + agent_name_ + "/t265/odom/sample", 1000, &ArgosAgent::OdometryCallback(), this);
  
  // Enable graceful termination of the node, e.g., when requested via CTRL+C
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGKILL, SignalHandler);

  // Setup PID  --- TODO Independant Class

  // Setup Pure Pursuit --- TODO Independant Class

};
ArgosAgent::~ArgosAgent() {};

int ArgosAgent::run() {

  ros::Duration timestep(0.05);
  
  // Main agent loop
  while (!g_request_shutdown && ros::ok()) {
    

    // //
    ros::spinOnce();
    timestep.sleep();
  
  return EXIT_SUCCESS;
};

void ArgosAgent::lookaheadCallback(const geometry_msgs::Pose::ConstPtr& lookahead_point) {
  lookahead_point_ = *lookahead_point;
  theta_ = ArgosAgent::getSteeringAngle();
}

void ArgosAgent::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry) {
  odometry_ = *odometry;
  double current_velocity = odometry.twist.twist.linear.x; 
  throttle_ = ArgosAgent::pidStep(current_velocity, desired_velocity_, time)
}

double ArgosAgent::pidStep(double current_velocity, double desired_velocity, ros::Duration timestamp) {
  ros::Duration dt = timestamp - prev_time_;
  double error = desired_velocity - current_velocity;
  double delta_error = (error - prev_error_) / dt;
  integral_ += error * dt;

  ROS_INFO_NAMED("PID", "Integral Value: ", integral_);


  throttle = p_ * error + d_ * delta_error + i_ * integral;

  prev_time_ = timestamp;
  prev_error_ = error;

  return throttle;
}

double ArgosAgent::getSteeringAngle() {
  double alpha = atan2(lookahead_point_.position.y, lookahead_point_.position.y);
  double theta = atan((2*0.35*sin(alpha))/(sqrt(pow(lookahead_point_.position.x,2)+ pow(lookahead_point_.position.y,2)));

  return (theta / 3.14159f * 180.0f)
}
