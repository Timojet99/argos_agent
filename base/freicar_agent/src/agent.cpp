/*
 * FreiCAR class - winter semester 2023/24
 * Do NOT distribute this code to anyone outside the FreiCAR project.
 */

#include "freicar_agent/agent.h"

#include <freicar_msgs/AgentCommand.h>
#include <freicar_msgs/AgentState.h>
#include <freicar_msgs/ControlCommand.h>
#include <freicar_msgs/ControlReport.h>
#include <freicar_msgs/Track.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <stdlib.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>

#include <cmath>
#include <csignal>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

Agent::Agent()
    : nh_private_("~"), tf_listener_(tf_buffer_), is_stopped_ext_(false), is_registered_(false), control_mode_(false) {
    // Receive parameters from the parameter server
    nh_private_.getParam("agent_name", agent_name_);
    nh_private_.getParam("global_frame", global_frame_);
    nh_private_.getParam("cmd_topic", cmd_topic_);
    nh_private_.getParam("agent_state_topic", agent_state_topic_);
    nh_private_.getParam("track_service", track_service_id_);
    nh_private_.getParam("requires_chaperone", requires_chaperone_);
    nh_private_.getParam("safety_radius", safety_radius_);
    nh_private_.getParam("safety_localization_time", safety_localization_time_);

    // TF parameters
    rear_axis_frame_ = agent_name_ + "/rear_axis";
    base_link_frame_ = agent_name_ + "/base_link";

    // Set up subscribers and publishers
    agent_cmd_sub_ = nh_.subscribe(cmd_topic_, 10, &Agent::ProcessCommand, this);
    control_report_sub_ =
        nh_.subscribe("/" + agent_name_ + "/control_report", 1000, &Agent::ControlReportCallback, this);
    state_pub_ = nh_.advertise<freicar_msgs::AgentState>(agent_state_topic_, 1);
    control_cmd_pub_ = nh_.advertise<freicar_msgs::ControlCommand>("/" + agent_name_ + "/control", 1000);
    control_mode_pub_ = nh_.advertise<std_msgs::Bool>("/" + agent_name_ + "/control_mode", 1000);

    // Enable graceful termination of the node, e.g., when requested via CTRL+C
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGKILL, SignalHandler);
}

Agent::~Agent() {}

int Agent::Run() {
    // Register this agent to be tracked by the chaperone
    if (requires_chaperone_ && !RegisterToChaperone()) {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Error since agent is not registered to the chaperone.");
        std::exit(EXIT_FAILURE);
    }

    // This update frequency is just a recommendation
    ros::Duration timestep(0.05);

    // Enable autonomous control
    SetControlMode(true);

    // Main agent loop:
    // The loop condition is very important for a timely graceful shutdown of this node. If the code below contains any
    // inner loops that take a long time to run, make sure they break quickly after this condition becomes false.
    int i = 0;
    while (!g_request_shutdown && ros::ok()) {
        // TODO: call any controller in here, but stop car immediately if a stop command was received (see callback).

        bool is_drive_ok = Drive(i++);

        // Make sure to publish state info periodically.
        if (!is_stopped_ext_) {
            PublishState(freicar_msgs::AgentState::STATE_IDLE);
        } else {
            PublishState(freicar_msgs::AgentState::STATE_STOPPED_EXT);
        }

        // REMINDER: ros::spin() or ros::spinOnce() are required for messages to be published and received!
        // If the code above contains any inner loops that take a long time to execute, run ros::spinOnce() inside
        // them as well. Otherwise a command may be missed.
        ros::spinOnce();
        timestep.sleep();

        if (!is_drive_ok || i == 75) {
            break;
        }
    }

    // Stop the car
    DoEmergencyStop();

    // Deregister this agent from the tracking of the chaperone
    if (requires_chaperone_ && !DeregisterFromChaperone()) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void Agent::SetControlMode(bool enable_control) {
    // Sends a message to the topic /control_mode to set the car to automatic control, enabling /control_cmd commands.
    // Note that the control_mode is automatically disabled if the hand remote is used.
    bool is_already_set = control_mode_ == enable_control;

    if (!is_already_set) {
        ros::Rate rate(10);
        std_msgs::Bool control_mode_msg;
        control_mode_msg.data = enable_control;
        int i = 0;
        while (control_mode_ != enable_control && ros::ok()) {
            ROS_INFO_STREAM_THROTTLE(0.5, "[" << agent_name_ << "] Trying to set control_mode to " << enable_control);
            control_mode_pub_.publish(control_mode_msg);
            rate.sleep();
            ros::spinOnce();
            // Avoid infinite loop if the car does not react
            if (g_request_shutdown && i++ == 10) {
                break;
            }
        }
    }

    if (!is_already_set && enable_control) {
        ROS_INFO_STREAM("[" << agent_name_ << "] Enabled automatic control");
    } else if (is_already_set && enable_control) {
        ROS_INFO_STREAM_THROTTLE(2.0, "[" << agent_name_ << "] Automatic control was already enabled");
    } else if (!is_already_set && !enable_control) {
        ROS_INFO_STREAM_THROTTLE(2.0, "[" << agent_name_ << "] Disabled automatic control");
    } else {
        ROS_INFO_STREAM_THROTTLE(2.0, "[" << agent_name_ << "] Automatic control was already disabled");
    }
}

void Agent::ControlReportCallback(const freicar_msgs::ControlReportConstPtr& control_report) {
    control_mode_ = control_report->control_mode;
}

void Agent::SendControlCommand(float throttle, float steering) {
    // This function should not be changed! If you believe that you have to adapt the code, talk to the TAs.

    // CAUTION: forward and reverse driving are inverted on some cars.
    if (agent_name_ == "freicar_2" || agent_name_ == "freicar_3" || agent_name_ == "freicar_6") {
        throttle *= -1;
    }

    freicar_msgs::ControlCommand control_cmd_msg;
    control_cmd_msg.header.frame_id = agent_name_;
    control_cmd_msg.header.stamp = ros::Time::now();
    control_cmd_msg.throttle = throttle;
    control_cmd_msg.steering = steering;
    control_cmd_pub_.publish(control_cmd_msg);
}

void Agent::DoEmergencyStop() {
    ROS_INFO_STREAM("[" << agent_name_ << "] Emergency stopping...");
    SetControlMode(true);  // For redundancy
    SendControlCommand(0.0, 0.0);
    SetControlMode(false);
}

bool Agent::GetPose(tf2::Stamped<tf2::Transform>& pose, std::string frame_id) {
    try {
        // Receive the current pose in the map coordinate system
        geometry_msgs::TransformStamped pose_tf_msg = tf_buffer_.lookupTransform(global_frame_, frame_id, ros::Time(0));

        // Check whether the transform is too old - > car is not being tracked anymore
        ros::Duration lag_transform = ros::Time::now() - pose_tf_msg.header.stamp;
        if (lag_transform.toSec() > safety_localization_time_) {
            ROS_WARN_STREAM("[" << agent_name_ << "] Localization too old: " << lag_transform.toSec() << " sec");
            return false;
        }

        // Parse the TF message
        tf2::convert(pose_tf_msg, pose);
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Failed to lookup car pose: " << ex.what());
        return false;
    }
}

bool Agent::GetMap() {
    // Add your team-specific solution for obtaining your map representation.
    return true;
}

bool Agent::IsOutsideMap(const tf2::Stamped<tf2::Transform>& agent_pose, float radius) {
    // Add your team-specific solution for checking whether the car is within the segmented map area.
    return false;
}

bool Agent::Drive(int index) {
    bool localization_ok = GetPose(rear_axis_pose_, rear_axis_frame_) && GetPose(base_link_pose_, base_link_frame_);

    // Safety check: if no car location can be obtained or the location is outside the drivable area by more than
    // the configured safety radius, perform an emergency stop
    if (!localization_ok || IsOutsideMap(base_link_pose_, safety_radius_)) {
        DoEmergencyStop();
        return false;
    }

    // Dummy values
    float steering = std::sin(2 * M_PI * index / 50) * 30;
    float throttle = 0.05;
    // CAUTION: freicar_6 needs slightly higher values for throttle
    if (agent_name_ == "freicar_6") {
        throttle = 0.085;
    }
    SendControlCommand(throttle, steering);

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Communication with the chaperone
///////////////////////////////////////////////////////////////////////////////
// THE CODE BELOW SHOULD ONLY BE CHANGED WITH CARE!
///////////////////////////////////////////////////////////////////////////////

void Agent::PublishState(uint8_t state) {
    /*
     * Generates and publishes an AgentState message with the given state on the given publisher.
     */
    freicar_msgs::AgentState state_msg;
    state_msg.header.frame_id = global_frame_;
    state_msg.header.stamp = ros::Time::now();

    state_msg.agent_id = agent_name_;

    // TODO: Set these fields once you have a plan
    // state_msg.current_plan.poses = plan_;
    // state_msg.current_plan.header = state_msg.header;
    // state_msg.plan_index = plan_index;

    assert(state == state_msg.STATE_IDLE || state == state_msg.STATE_DRIVING || state == state_msg.STATE_STOPPED_EXT ||
           state == state_msg.STATE_STOPPED_SELF);
    state_msg.state = state;

    state_pub_.publish(state_msg);
}

bool Agent::RegisterToChaperone() {
    /*
     * Sends a track request to the chaperone. Will terminate the program if the registration was unsuccessful.
     */
    if (is_registered_) {
        ROS_WARN_STREAM("[" << agent_name_ << "] Error registering to the chaperone since it is already registered.");
        return true;
    }

    ros::ServiceClient client = nh_.serviceClient<freicar_msgs::Track>(track_service_id_);

    freicar_msgs::Track track;
    track.request.track = true;

    // Make sure to always set this to the car's frame ID and that a transform to the global map frame is available a
    // the time of registration!
    track.request.agent_id = agent_name_;

    // TODO: If the agent already knows the plan, set this to true and publish it via the State message.
    track.request.knows_plan = false;

    if (!client.call(track)) {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Error registering to chaperone: could not connect to chaperone.");
        // Immediately exit if chaperone can't be reached
        std::exit(EXIT_FAILURE);
    }

    if (track.response.success) {
        ROS_INFO_STREAM("[" << agent_name_ << "] Successfully registered to the chaperone.");
        is_registered_ = true;
    } else {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Error registering to the chaperone!");
        // Exit if the registration was unsuccessful
        std::exit(EXIT_FAILURE);
    }

    return track.response.success;
}

bool Agent::DeregisterFromChaperone() {
    /*
     * Deregisters the agent from the chaperone. Should be called before the node shuts down.
     */
    if (!is_registered_) {
        ROS_WARN_STREAM("[" << agent_name_ << "] Error deregistering from chaperone since it was not registered");
        return true;
    }

    ros::ServiceClient client = nh_.serviceClient<freicar_msgs::Track>(track_service_id_);

    freicar_msgs::Track track;

    track.request.track = false;
    track.request.agent_id = agent_name_;

    if (!client.call(track)) {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Error deregistering from chaperone: could not connect to chaperone.");
    }

    if (track.response.success) {
        ROS_INFO_STREAM("[" << agent_name_ << "] Successfully deregistered from the chaperone.");
        is_registered_ = false;
    } else {
        ROS_ERROR_STREAM("[" << agent_name_ << "] Failed to deregister from the chaperone!");
    }

    return track.response.success;
}

void Agent::ProcessCommand(const freicar_msgs::AgentCommand& cmd) {
    /* This is a basic callback for processing agent commands (for now just stop and resume). The Command includes more
     * information such as the issuing authority and a reason, both are not stored in this example.
     */

    // ignore commands not meant for us
    if (cmd.target_agent != agent_name_) {
        return;
    }

    if (cmd.command == cmd.CMD_STOP) {
        ROS_INFO_STREAM("[" << agent_name_ << "] Received a stop command from " << cmd.authority);
        DoEmergencyStop();
        // TODO: remember at which step of the plan the agent stopped
        is_stopped_ext_ = true;
    } else if (cmd.command == cmd.CMD_RESUME) {
        ROS_INFO_STREAM("[" << agent_name_ << "] Received a resume command from " << cmd.authority);
        SetControlMode(true);
        // TODO: resume executing the plan, i.e., following the path
        is_stopped_ext_ = false;
    } else {
        ROS_WARN_STREAM("[" << agent_name_ << "] Received unknown command: " << cmd.command << " -- IGNORING");
    }
}

///////////////////////////////////////////////////////////////////////////////
// Static functions
///////////////////////////////////////////////////////////////////////////////

void Agent::SignalHandler(int signal) {
    /* Signal handler that allows us to do stuff (e.g., deregister from chaperone) before the node exits.
     * Instead of exiting right here, a flag is set that interrupts the main loop below.
     * Actions that should run before shutdown can be added here or below the main loop (for easy access to the
     * NodeHandle). See:
     * https://answers.ros.org/question/27655/what-is-the-correct-way-to-do-stuff-before-a-node-is-shutdown/
     */
    switch (signal) {
        case 2:
            ROS_INFO_STREAM("Received SIGINT, getting ready to exit...");
            break;
        case 9:
            ROS_INFO_STREAM("Received SIGKILL, getting ready to exit...");
            break;
        default:
            ROS_WARN_STREAM("Signal handler received an unexpected signal, ignoring.");
            return;
    }

    // set a signal-safe flag to indicate that this node wants to shut down
    g_request_shutdown = 1;
}
