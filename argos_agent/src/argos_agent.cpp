/*
 * ArgosAgent Class - Master Project Timo Müller SS25
 * 
 */

#include "argos_agent/argos_agent.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
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
  nh_private_.getParam("cmd_topic", cmd_topic_);
  nh_private_.getParam("agent_state_topic", _);
  nh_private_.getParam("proportional", proportional_);
  nh_private_.getParam("differential", differential_);
  nh_private_.getParam("integral", integral_);
  nh_private_.getParam("desired_velocity", desired_velocity_);
  nh_private_.getParam("lookahead_distance" lookahead_distance_);
  
  // TF parameters
  std::string rear_axis_frame_ = agent_name_ + "/rear_axis";
  std::string base_link_frame_ = agent_name_ + "/base_link";

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

  // Setup PID
  PID pid(proportional_, differential_ integral_);

  // Setup Pure Pursuit --- TODO

};
ArgosAgent::~ArgosAgent() {};

int ArgosAgent::Run() {

  ros::Duration timestep(0.05);
  
  // Main agent loop
  while (!g_request_shutdown && ros::ok()) {


    // //
    ros::spinOnce();
    timestep.sleep();
  }
  return EXIT_SUCCESS;
};

void ArgosAgent::LookaheadCallback(const geometry_msgs::Pose::ConstPtr& lookahead_point_) {
  float x = lookahead_point->position.x;
  float y = lookahead_point->position.y;

  
}
