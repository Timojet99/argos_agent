#ifndef FREICAR_CHAPERONE_INCLUDE_TRACKEDAGENT_H_
#define FREICAR_CHAPERONE_INCLUDE_TRACKEDAGENT_H_

#include <freicar_msgs/AgentState.h>
#include <freicar_msgs/ControlReport.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Collision.h"
#include "Footprint.h"
#include "GeoFence.h"
#include "MovingAverage.h"

class TrackedAgent {
    // A agent that is being watched by the chaperone.
   public:
    TrackedAgent();
    tf::TransformListener* tfl_ptr;
    tf::TransformBroadcaster tf_broadcaster;

    std::string name;
    bool knows_plan{};
    ros::Subscriber state_sub;
    ros::Subscriber control_mode_sub;
    ros::Publisher control_mode_pub;
    ros::Publisher control_command_pub;
    bool valid_corridor = false;

    // indicates whether this agent was stopped by the chaperone to prevent a collision
    bool stopped = false;

    // TODO: for the external force stop, add a field 'bool acknowledged_stop' and set it to true if the agent sent a
    // state where it ack'd that it will stop by external command. if the field does not become true within ~400ms, stop
    // by force.

    // if this agent was stopped because of a predicted collision with another agent, this field will hold the name of
    // that agent
    std::string stopped_by;
    ros::Time stopped_at;

    uint state{};
    std::vector<geometry_msgs::PoseStamped> plan;
    uint plan_index{};

    std::vector<Footprint> corridor;

    tf::StampedTransform last_tf;
    bool valid_tf = false;

    MovingAverage<float, float, 10> mv_vel_x;
    MovingAverage<float, float, 10> mv_vel_y;
    MovingAverage<float, float, 10> mv_vel_yaw;

    ros::Time last_state_update;

    // settings:
    std::string global_frame;
    float car_width{};
    float car_len{};
    float lookahead_duration{};
    float sampling_timestep{};
    bool automatic_resume;
    ros::Duration resume_delay;
    bool force_stop_enabled;
    ros::Duration force_stop_delay;
    bool control_mode;

    TrackedAgent(const ros::NodeHandle& nh_, tf::TransformListener* tfl, const std::string& name, bool will_know_plan);

    bool should_force_stop();

    void update_agent_state(const freicar_msgs::AgentState& new_state);

    void update_tf();

    void update_velocity(const tf::StampedTransform& new_tf);

    void update_corridor();

    Collision will_collide(TrackedAgent* other_agent);

    Collision would_collide_if_stopped(TrackedAgent* other_agent, bool this_stopped, bool other_stopped);

    visualization_msgs::MarkerArray generate_corridor_markers_msg(std_msgs::ColorRGBA vis_color) const;

    Footprint generate_footprint() const;

    Footprint generate_footprint(const tf::Pose& pose) const;

    double get_linear_speed();
    bool should_resume(TrackedAgent* other_agent);
    bool should_resume(GeoFence* map_boundary) const;
    void refresh_data();

    void control_report_callback(const freicar_msgs::ControlReport& control_report);

    void force_stop();
};

#endif  // FREICAR_CHAPERONE_INCLUDE_TRACKEDAGENT_H_
