/*
 * FreiCAR class - winter semester 2023/24
 * Do NOT distribute this code to anyone outside the FreiCAR project.
 */

#pragma once

#include <freicar_msgs/AgentCommand.h>
#include <freicar_msgs/ControlReport.h>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <string>

class Agent {
   public:
    Agent();
    ~Agent();

    int Run();

   private:
    static void SignalHandler(int signal);

    // Communication with chaperone
    void PublishState(uint8_t state);
    bool RegisterToChaperone();
    bool DeregisterFromChaperone();
    void ProcessCommand(const freicar_msgs::AgentCommand& cmd);
    bool is_stopped_ext_;
    bool is_registered_;

    // Driving commands
    void SetControlMode(bool enable_control);
    void SendControlCommand(float throttle, float steering);
    void DoEmergencyStop();
    void ControlReportCallback(const freicar_msgs::ControlReportConstPtr& control_report);

    // Navigation & control related
    bool GetPose(tf2::Stamped<tf2::Transform>& pose, std::string frame_id);
    bool GetMap();
    bool IsOutsideMap(const tf2::Stamped<tf2::Transform>& agent_pose, float radius);
    bool Drive(int index);

    // Basic parameters
    std::string agent_name_;
    std::string global_frame_;
    std::string cmd_topic_;
    std::string agent_state_topic_;
    std::string track_service_id_;
    bool requires_chaperone_;  // If true, the agent can only start after registration to the chaperone.

    // Various variables
    bool control_mode_;  // If true, the agent node can control the RC car.
    float safety_radius_;
    float safety_localization_time_;

    // TF parameters and variables
    std::string rear_axis_frame_;
    std::string base_link_frame_;
    tf2::Stamped<tf2::Transform> rear_axis_pose_;
    tf2::Stamped<tf2::Transform> base_link_pose_;

    // ROS infrastructure
    ros::NodeHandle nh_, nh_private_;
    ros::Subscriber agent_cmd_sub_, control_report_sub_;
    ros::Publisher state_pub_, control_cmd_pub_, control_mode_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
