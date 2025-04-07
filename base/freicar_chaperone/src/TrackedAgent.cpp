#include "TrackedAgent.h"

#include <Footprint.h>
#include <GeoFence.h>
#include <freicar_msgs/AgentState.h>
#include <freicar_msgs/ControlCommand.h>
#include <freicar_msgs/ControlReport.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "Collision.h"

TrackedAgent::TrackedAgent(const ros::NodeHandle& nh_, tf::TransformListener* tfl_, const std::string& name_,
                           bool will_know_plan) {
    /*
     * Constructor: create a TrackedAgent. Note: Will NOT subscribe to the state, do this after creating the agent and
     * register this agent's state callback function.
     */

    name = name_;
    tfl_ptr = tfl_;

    knows_plan = will_know_plan;
    last_state_update = ros::Time(0);

    nh_.param("global_frame", global_frame, std::string("map"));
    nh_.param("lookahead_duration", lookahead_duration, static_cast<float>(2.5));
    nh_.param("sampling_timestep", sampling_timestep, static_cast<float>(0.15));
    nh_.param("car_width", car_width, static_cast<float>(0.3));
    nh_.param("car_len", car_len, static_cast<float>(0.6));
    nh_.param("automatic_resume", automatic_resume, static_cast<bool>(true));
    nh_.param("force_stop_enabled", force_stop_enabled, static_cast<bool>(true));

    float force_stop_delay_fl;
    nh_.param("force_stop_delay", force_stop_delay_fl, static_cast<float>(0.5));
    force_stop_delay = ros::Duration(force_stop_delay_fl);

    // TODO: tune the resume delay
    float resume_delay_fl;
    nh_.param("resume_delay", resume_delay_fl, static_cast<float>(2.0));
    resume_delay = ros::Duration(resume_delay_fl);

    mv_vel_x.clear();
    mv_vel_y.clear();
    mv_vel_yaw.clear();

    // initialize tf
    update_tf();
}

TrackedAgent::TrackedAgent() {
    // TODO: this empty constructor is required for storing TrackedAgents in a map, but is it correct this way?
}

bool TrackedAgent::should_force_stop() {
    /*
     * Determines if the agent should be stopped by sending commands to the hardware directly, e.g. because it's
     * unresponsive. Returns true if all of the following conditions are met:
     * - Agent is currently stopped and the set force_stop_delay has passed since the agent was stopped
     * - Force stopping is enabled
     * - The last known state of this agent wasn't STATE_STOPPED_EXT or STATE_STOPPED_SELF
     */
    return (stopped && force_stop_enabled &&
            !(state == freicar_msgs::AgentState::STATE_STOPPED_EXT ||
              state == freicar_msgs::AgentState::STATE_STOPPED_SELF) &&
            std::abs((ros::Time::now() - stopped_at).toSec()) > force_stop_delay.toSec());
}

void TrackedAgent::force_stop() {
    /*
     * Issues commands to this car's hardware controller directly to stop driving without cooperation from the
     * agent node (e.g. in case it's unresponsive). This is done by first sending a control command which sets the
     * steering and throttle to 0.0, and then sending up to five commands to disable automatic control.
     */
    freicar_msgs::ControlCommand control_command_msg;
    control_command_msg.header.frame_id = name;
    control_command_msg.header.stamp = ros::Time::now();
    control_command_msg.throttle = 0.0;
    control_command_msg.steering = 0.0;
    control_command_pub.publish(control_command_msg);

    ros::spinOnce();

    std_msgs::Bool control_mode_msg;
    // control mode false -> disable automatic control
    control_mode_msg.data = false;
    int try_cnt = 5;
    while (control_mode && ros::ok() && try_cnt >= 0) {
        ROS_INFO_THROTTLE(0.5, "Trying to disable automatic control on car %s", name.c_str());
        control_mode_pub.publish(control_mode_msg);
        ros::spinOnce();
        ros::Duration(sampling_timestep).sleep();
        ros::spinOnce();
        try_cnt--;
    }

    if (control_mode) {
        ROS_WARN("Failed to disable automatic control on car %s while force stopping!", name.c_str());
    }
}

void TrackedAgent::refresh_data() {
    /*
     * This function is intended to be periodically called by the main loop. It checks whether the state and tf data is
     * current (i.e. whether the car this agent is tracking is updating the chaperone frequently enough). If the state
     * or the tf is outdated, update the tf and extrapolate the corridor from the last tfs and the speed..
     */
    // TODO: check minimum update frequency against real agents
    if (!knows_plan || !valid_tf || (ros::Time::now() - last_tf.stamp_) > ros::Duration(sampling_timestep * 2)) {
        update_tf();
    }
    if (!knows_plan || !valid_corridor ||
        (ros::Time::now() - last_state_update) > ros::Duration(sampling_timestep * 2)) {
        update_corridor();
    }
}

void TrackedAgent::update_agent_state(const freicar_msgs::AgentState& new_state) {
    /* callback to update the agent state. Will also try to retrieve a new tf and update the velocity estimates
     */

    // auto i = agents.find(state.agent_id);
    if (new_state.agent_id != name) {
        ROS_WARN_THROTTLE(
            1.0,
            "Agent %s tracked by chaperone received a state message from other agent %s on topic %s, discarding...",
            name.c_str(), new_state.agent_id.c_str(), state_sub.getTopic().c_str());
        return;
    }

    if (knows_plan && !new_state.current_plan.poses.empty() && new_state.plan_index >= 0) {
        // received a valid plan: store it
        // TODO: should we validate the received plan by checking that the current plan pose is near the actual
        // position?
        plan = new_state.current_plan.poses;
        plan_index = new_state.plan_index;

        tf::Pose p;
        p.setIdentity();
        p.setOrigin(tf::Vector3(plan[plan_index].pose.position.x, plan[plan_index].pose.position.y, 0.0));
        tf_broadcaster.sendTransform(tf::StampedTransform(p, ros::Time::now(), global_frame, "/" + name + "_plan_idx"));

    } else {
        // no valid plan or agent indicated that it doesn't know its plan and thus wants to be tracked by extrapolating
        // velocity
        plan.clear();
        plan_index = 0;
    }

    if (new_state.state == new_state.STATE_IDLE || new_state.state == new_state.STATE_DRIVING ||
        new_state.state == new_state.STATE_STOPPED_SELF || new_state.state == new_state.STATE_STOPPED_EXT) {
        // agent sent a valid state ID, store it:
        state = new_state.state;
    } else {
        ROS_WARN_THROTTLE(1.0, "Agent %s sent invalid state id %d on topic %s", name.c_str(), new_state.state,
                          this->state_sub.getTopic().c_str());
    }

    last_state_update = ros::Time::now();

    update_tf();
    update_corridor();
}

void TrackedAgent::control_report_callback(const freicar_msgs::ControlReport& control_report) {
    control_mode = control_report.control_mode;
    // approximate velocity from wheel rpm (line from freicar_odometry)
    // current_v = (-1 * float(control_report->rpm) / 60.f) * (2.f * M_PI * 0.05);
}

void TrackedAgent::update_tf() {
    /*
     * Tries to retrieve a new pose from the TF listener for this agent. Estimates current
     * Velocity by comparing to previous tf if it exists.
     */

    tf::StampedTransform new_tf;

    if (tfl_ptr->canTransform(global_frame, name + "/base_link", ros::Time(0))) {
        tfl_ptr->lookupTransform(global_frame, name + "/base_link", ros::Time(0), new_tf);
    } else {
        ROS_WARN("Tracked agent %s unable to get pose", name.c_str());
        return;
    }

    // set z to 0
    new_tf.setOrigin(tf::Vector3(new_tf.getOrigin().getX(), new_tf.getOrigin().getY(), 0.0));

    if (!valid_tf) {
        // this is the first tf received, store it and exit:
        last_tf = new_tf;
        valid_tf = true;
        return;
    }

    // previous tf was valid, check if the received tf is actually newer:
    if (new_tf.stamp_ > last_tf.stamp_) {
        // update velocity estimate from difference
        update_velocity(new_tf);
        // store the new tf
        last_tf = new_tf;
    }
}

void TrackedAgent::update_velocity(const tf::StampedTransform& new_tf) {
    /*
     * Update this agent's moving averages for linear and angular velocity by computing the difference
     * between the previous and the new given tf. Assumptions: previous tf was valid and new tf is actually
     * newer than previous.
     */

    assert(valid_tf && new_tf.stamp_ > last_tf.stamp_);

    double time_diff = std::abs((new_tf.stamp_ - last_tf.stamp_).toSec());

    if (time_diff > 1.0) {
        ROS_WARN("Agent %s encountered a jump between TFs larger than 1s (%f s), flushing velocity estimates...",
                 name.c_str(), time_diff);
        mv_vel_x.clear();
        mv_vel_y.clear();
        mv_vel_yaw.clear();
        return;
    }

    // linear velocities in x and y directions in global frame:
    mv_vel_x.new_entry((new_tf.getOrigin().x() - last_tf.getOrigin().x()) / time_diff);
    mv_vel_y.new_entry((new_tf.getOrigin().y() - last_tf.getOrigin().y()) / time_diff);

    // get yaw from difference tf:
    tf::Transform new_in_old = last_tf.inverse() * new_tf;
    double yaw, pitch, roll;
    tf::Matrix3x3 m(new_in_old.getRotation());
    m.getRPY(roll, pitch, yaw);
    mv_vel_yaw.new_entry(yaw / time_diff);
}

double TrackedAgent::get_linear_speed() {
    // returns the linear speed of the agent
    return sqrt(pow(mv_vel_x.mean(), 2) + pow(mv_vel_y.mean(), 2));
}

void TrackedAgent::update_corridor() {
    /*
     * Update the corridor using the velocity estimate and, if provided, the current plan.
     */
    // TODO: handle stale data somehow. Maybe try to get new tfs and ignore the plan in this case
    // other idea: in the main loop, frequently check all agents if the data is fresh. if it's not, enter
    // backup mode to use only tf estimates
    // possibly the safest way: don't rely on the plans at all, but store two corridors: one from the plan and one
    // purely from tf motion estimates, then cross-check

    // do not update the corridor if the agent is stopped, want to keep the last known plan
    if (stopped) return;

    corridor.clear();

    assert(sampling_timestep > 0);

    uint target_corridor_size = ceil(lookahead_duration / sampling_timestep) + 1;

    tf::Pose current_pose((tf::Pose)last_tf);

    double speed = get_linear_speed();

    if (plan.empty() || speed == 0.0 || state == freicar_msgs::AgentState::STATE_IDLE ||
        state == freicar_msgs::AgentState::STATE_STOPPED_EXT) {
        // no plan or not driving: simulate driving with the same velocity
        tf::Quaternion yaw_increment;
        yaw_increment.setRPY(0.0, 0.0, mv_vel_yaw.mean() * sampling_timestep);
        double x_increment = mv_vel_x.mean() * sampling_timestep;
        double y_increment = mv_vel_y.mean() * sampling_timestep;
        // ROS_INFO("yaw %f x %f y %f", mv_vel_yaw.mean() * sampling_timestep, x_increment, y_increment);
        corridor.push_back(generate_footprint(current_pose));
        while (corridor.size() < target_corridor_size) {
            tf::Quaternion new_rotation = yaw_increment * current_pose.getRotation();
            current_pose.setRotation(new_rotation);

            tf::Vector3 new_origin = current_pose.getOrigin();
            new_origin.setX(new_origin.getX() + x_increment);
            new_origin.setY(new_origin.getY() + y_increment);
            current_pose.setOrigin(new_origin);

            corridor.push_back(generate_footprint(current_pose));
        }
        valid_corridor = true;
        return;
    } else if (plan.size() == 1) {
        // special case: plan contains exactly one pose. Assume the agent stops there.
        tf::poseMsgToTF(plan[plan_index].pose, current_pose);
        while (corridor.size() < target_corridor_size) {
            corridor.push_back(generate_footprint(current_pose));
        }
        valid_corridor = true;
        return;
    }

    int next_wpt_idx = plan_index + 1;
    tf::Vector3 next_wpt(plan[next_wpt_idx].pose.position.x, plan[next_wpt_idx].pose.position.y, 0.0);
    bool finished = false;
    double total_distance = lookahead_duration * speed;
    double distance_covered = 0.0;

    // set the first corridor footprint to the first position in the plan
    current_pose.setOrigin(tf::Vector3(plan[plan_index].pose.position.x, plan[plan_index].pose.position.y, 0.0));
    tf::Vector3 diff = next_wpt - current_pose.getOrigin();
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, atan2(diff.y(), diff.x()));
    current_pose.setRotation(q);
    corridor.push_back(generate_footprint(current_pose));

    while (next_wpt_idx < plan.size() && distance_covered < total_distance) {
        double delta_dist = speed * sampling_timestep;
        double dist2wpt = (next_wpt - current_pose.getOrigin()).length();

        // walk along the path for delta_dist
        while (dist2wpt < delta_dist) {
            if (next_wpt_idx == plan.size() - 1) {
                // if there is less distance of path left than delta_dist, we're done
                finished = true;
                break;
            }
            current_pose.setOrigin(next_wpt);
            next_wpt_idx++;

            next_wpt = tf::Vector3(plan[next_wpt_idx].pose.position.x, plan[next_wpt_idx].pose.position.y, 0.0);
            delta_dist -= dist2wpt;
            distance_covered += dist2wpt;
            dist2wpt = (next_wpt - current_pose.getOrigin()).length();
        }

        if (finished) break;

        // now the end of this step will lie on the current path segment, so we get the next pose by going in the
        // direction of the next wpt
        tf::Vector3 diff = next_wpt - current_pose.getOrigin();
        current_pose.setOrigin(current_pose.getOrigin() + (delta_dist / dist2wpt) * diff);

        tf::Quaternion q;
        q.setRPY(0.0, 0.0, atan2(diff.y(), diff.x()));
        current_pose.setRotation(q);

        distance_covered += delta_dist;

        corridor.push_back(generate_footprint(current_pose));
    }

    // fill remaining corridor with the same pose
    while (corridor.size() < target_corridor_size) {
        corridor.push_back(corridor[corridor.size() - 1]);
    }

    assert(corridor.size() == target_corridor_size);

    valid_corridor = true;
}

Footprint TrackedAgent::generate_footprint(const tf::Pose& pose) const {
    /*
     * Generate a car footprint in the given pose.
     */

    // only consider rotation around the yaw axis. The footprint is 2d.
    tf::Matrix3x3 m;
    m.setRotation(pose.getRotation());
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf::Pose pose_flat_rotation(pose);
    pose_flat_rotation.setRotation(q);

    // get car corner points in global frame
    std::vector<Eigen::Vector3d> points{
        static_cast<Eigen::Vector3d>(pose_flat_rotation * tf::Point(car_len / 2, car_width / 2, 0.0)),
        static_cast<Eigen::Vector3d>(pose_flat_rotation * tf::Point(car_len / 2, -car_width / 2, 0.0)),
        static_cast<Eigen::Vector3d>(pose_flat_rotation * tf::Point(-car_len / 2, -car_width / 2, 0.0)),
        static_cast<Eigen::Vector3d>(pose_flat_rotation * tf::Point(-car_len / 2, car_width / 2, 0.0))};

    return Footprint(points);
}

Footprint TrackedAgent::generate_footprint() const {
    // generate a footprint in the frame of the agent
    tf::Pose identity;
    identity.setIdentity();
    return generate_footprint(identity);
}

Collision TrackedAgent::will_collide(TrackedAgent* other_agent) {
    /* returns true if the predicted corridors of this agent and the other agent will collide
     * Assumes that the corridors have the same number of footprints and the timestep between the footprints is the
     * same. Under these assumptions, the method check if the Nth footprints of each corridor overlap.
     *
     * If a collision is predicted, more information about it will be provided by writing into the Collision object.
     * */

    // consider the 'stopped' field of both agents.
    return would_collide_if_stopped(other_agent, stopped, other_agent->stopped);
}

Collision TrackedAgent::would_collide_if_stopped(TrackedAgent* other_agent, bool this_stopped, bool other_stopped) {
    /* returns true if the predicted corridors of this agent and the provided other agents would collide given that one
     * or the other agent has been stopped (i.e. stands still at the first pose in corridor)
     * Assumes that the corridors have the same number of footprints and the timestep between the footprints is the
     * same. Under these assumptions, the method check if the Nth footprints of each corridor overlap.
     *
     * Will return information on the collision. The collision is only valid if the field 'impending' is set to true.
     * */

    // make sure that both corridors exist and have the same length (fixed timestep and lookahead duration should always
    // produce same-length corridors
    assert(!corridor.empty());
    assert(corridor.size() == other_agent->corridor.size());

    Collision collision;
    collision.impending = false;

    if (!this_stopped && !other_stopped) {
        // both driving
        for (size_t i = 0; i < corridor.size(); i++) {
            if (corridor[i].intersect(other_agent->corridor[i])) {
                collision.collision_index_1 = i;
                collision.collision_index_2 = i;
                collision.impending = true;
            }
        }

    } else if (!this_stopped && other_stopped) {
        // i am driving, other is stopped
        Footprint other_fp = other_agent->generate_footprint(other_agent->last_tf);
        for (size_t i = 0; i < corridor.size(); i++) {
            if (corridor[i].intersect(other_fp) || corridor[i].intersect(other_agent->corridor[0])) {
                collision.collision_index_1 = i;
                collision.collision_index_2 = 0;
                collision.impending = true;
            }
        }

    } else if (this_stopped && !other_stopped) {
        // i am stopped, other is driving
        Footprint this_fp = generate_footprint(last_tf);
        for (size_t i = 0; i < corridor.size(); i++) {
            if (this_fp.intersect(other_agent->corridor[i]) || corridor[0].intersect(other_agent->corridor[i])) {
                collision.collision_index_1 = 0;
                collision.collision_index_2 = i;
                collision.impending = true;
            }
        }
    } else {
        // both stopped: check if first poses in corridors or current poses intersect
        Footprint other_fp = other_agent->generate_footprint(other_agent->last_tf);
        Footprint this_fp = generate_footprint(last_tf);
        if (this_fp.intersect(other_fp) || corridor[0].intersect(other_agent->corridor[0])) {
            collision.collision_index_1 = 0;
            collision.collision_index_2 = 0;
            collision.impending = true;
        }
    }

    if (collision.impending) {
        collision.first_predicted = ros::Time::now();
        collision.agent_1 = name;
        collision.agent_2 = other_agent->name;
    }

    return collision;
}

bool TrackedAgent::should_resume(TrackedAgent* other_agent) {
    /* Checks if this agent can resume, given that it has been stopped before due to an impending collision. If the
     * agent isn't stopped, will always return false. If the provided 'other_agent' isn't the reason this agent was
     * originally stopped, will also return false
     *
     * Will consider the following conditions:
     * - Whether the danger of colliding with the agent it originally stopped for is gone (will NOT check for collisions
     * with other agents)
     * - Whether the resume_delay since the agent has been stopped has passed
     * - Whether the automatic resuming is enabled
     * //TODO: idea: include a 'safe distance' that must be reached to all other agents before the agent can resume?
     * */

    if (!stopped || !automatic_resume || other_agent->name != stopped_by) {
        // agent not stopped, automatic resume disabled or the provided agent isn't the stopping reason
        return false;
    }

    ros::Duration time_since_stop = ros::Time::now() - stopped_at;
    if (time_since_stop < resume_delay) {
        // not enough time passed since the stop
        return false;
    }

    // check if the collision will happen if this agent is allowed to continue
    Collision collision = would_collide_if_stopped(other_agent, false, other_agent->stopped);
    if (collision.impending) {
        // still in danger of colliding
        return false;
    }

    // The agent is safe to resume
    return true;
}

bool TrackedAgent::should_resume(GeoFence* map_boundary) const {
    /* Checks if this agent can resume, given that it has been stopped before due to being outside the map boundary. If
     * the agent isn't stopped, will always return false. If the agent has been stopped for a different reason, e.g. an
     * impending collision, it will also always return false (even if the collision isn't a danger anymore, this only
     * checks the map boundary).
     *
     * Will consider the following conditions:
     * - Whether the agent was stopped by the map boundary, indicated by the field stopped_by=="MAP_BOUNDARY"
     * - Whether the agent is back inside the map
     * - Whether the resume_delay since the agent has been stopped has passed
     * - Whether the automatic resuming is enabled
     * //TODO: idea: include a 'safe distance' that must be reached to all other agents before the agent can resume?
     * */

    if (!stopped || !automatic_resume || stopped_by != map_boundary->STOPPING_REASON) {
        // agent not stopped, automatic resume disabled or this agent wasn't stopped by the map boundary
        return false;
    }

    ros::Duration time_since_stop = ros::Time::now() - stopped_at;
    if (time_since_stop < resume_delay) {
        // not enough time passed since the stop
        return false;
    }

    // check if the collision will happen if this agent is allowed to continue

    if (!map_boundary->inside(last_tf)) {
        // still outside boundary
        return false;
    }

    // The agent is safe to resume
    return true;
}

visualization_msgs::MarkerArray TrackedAgent::generate_corridor_markers_msg(std_msgs::ColorRGBA vis_color) const {
    /* Generate a MarkerArray Message for visualizing the predicted corridor of this agent in rviz.
     * Will use the provided color. Make sure the alpha is set!
     */
    visualization_msgs::MarkerArray markers;

    for (const Footprint& footprint : this->corridor) {
        markers.markers.push_back(footprint.createVisMarker(this->global_frame, ros::Time::now().nsec, vis_color));
    }

    return markers;
}
