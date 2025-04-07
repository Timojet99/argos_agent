#include "Chaperone.h"

#include <freicar_msgs/AgentCommand.h>
#include <freicar_msgs/ControlCommand.h>
#include <std_msgs/Bool.h>

Chaperone::Chaperone(const ros::NodeHandle& nh_) {
    nh = nh_;

    cmd_pub = nh.advertise<freicar_msgs::AgentCommand>("/freicar_agent_commands", 40);
    track_service = nh.advertiseService("/freicar_chaperone_track_srv", &Chaperone::handle_track_request, this);
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("corridors", 1);

    nh.param("global_frame", global_frame, std::string("map"));
    nh.param("lookahead_duration", lookahead_duration, static_cast<float>(2.5));
    nh.param("sampling_timestep", sampling_timestep, static_cast<float>(0.15));
    nh.param("car_width", car_width, static_cast<float>(0.3));
    nh.param("car_len", car_len, static_cast<float>(0.6));
    nh.param("resume_delay", resume_delay, static_cast<float>(2.0));

    map_boundary = GeoFence(nh);
    map_boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>("map_boundary", 1, true);
    map_boundary_pub.publish(map_boundary.get_marker_msg());
}

void Chaperone::run() {
    /*
     * Run the chaperone's main loop until ros wants to exit.
     */

    ros::Duration timestep(sampling_timestep);
    ROS_INFO("Chaperone ready to receive track requests");
    while (ros::ok()) {
        ROS_DEBUG_THROTTLE(10.0, "Chaperone: number of currently tracked agents: %lu", agents.size());

        update_all_agents();
        check_boundary();

        check_collisions();
        check_and_execute_force_stops();

        visualize_corridors(agents, marker_pub);

        ros::spinOnce();
        timestep.sleep();
    }
}

void Chaperone::update_all_agents() {
    /* update the TFs and corridors of all tracked agents with stale data by calling their tick functions.
     */
    for (auto& agent : agents) {
        agent.second.refresh_data();
    }
}

void Chaperone::check_boundary() {
    /*
     * Check if all agents are within the map boundary. if they are not, send them a stop command. If a stopped agent
     * is back within the map boundary, resume that agent.
     */
    for (auto& agent : agents) {
        bool agent_inside = map_boundary.inside(agent.second.last_tf);
        // if (!agent_inside && !agent.second.stopped) {
        if (!agent.second.stopped && !agent_inside) {
            // agent is outside map and not yet stopped
            ROS_INFO("Agent %s found outside the map boundary, will be stopped.", agent.second.name.c_str());
            suspend_agent(agent.second.name, freicar_msgs::AgentCommand::OUT_OF_BOUNDS);
            agent.second.stopped_by = map_boundary.STOPPING_REASON;
        }
        if (agent.second.should_resume(&map_boundary)) {
            // agent is currently stopped due to having been outside the boundary, but now it's inside. Resume.
            ROS_INFO("Agent %s is back inside the map boundary, will be resumed.", agent.second.name.c_str());
            resume_agent(agent.second.name);
        }
    }
}

void Chaperone::check_collisions() {
    /*
     * Cross-check all registered agents for future collisions with each other. Will also  try to prevent any
     * detected collisions.
     */

    // get a list of agent names so it's easier to iterate over them in the nested loop below.
    std::vector<std::string> agent_names;
    for (auto& this_agent : agents) {
        // only consider agents with a valid corridor
        if (!this_agent.second.valid_corridor) continue;
        agent_names.push_back(this_agent.first);
    }

    // One or zero agents: no collisions possible
    if (agent_names.size() <= 1) return;

    // iterate over the list of agents, check for collisions of the current agent against all other agents which haven't
    // been checked against this agent.
    for (size_t i = 0; i < agent_names.size() - 1; i++) {
        TrackedAgent* this_agent = &agents[agent_names[i]];

        for (size_t j = i + 1; j < agent_names.size(); j++) {
            TrackedAgent* other_agent = &agents[agent_names[j]];
            Collision collision = this_agent->will_collide(other_agent);

            if (collision.impending) {
                ROS_INFO("Chaperone predicts collision between agent %s and %s in %lu ticks.", this_agent->name.c_str(),
                         other_agent->name.c_str(), std::max(collision.collision_index_1, collision.collision_index_2));

                // show the collision in rviz and try to prevent it
                visualize_collision(collision);
                prevent_collision(collision);

            } else {
                // check if we can resume either of the agents
                if (this_agent->should_resume(other_agent)) {
                    resume_agent(this_agent->name);
                }
                if (other_agent->should_resume(this_agent)) {
                    resume_agent(other_agent->name);
                }
            }
        }
    }
}

void Chaperone::prevent_collision(const Collision& collision) {
    TrackedAgent* agent_1 = &agents[collision.agent_1];
    TrackedAgent* agent_2 = &agents[collision.agent_2];

    // check if the collision can be prevented by stopping either of the agents
    Collision collision_if_1_stops = agent_1->would_collide_if_stopped(agent_2, true, agent_2->stopped);
    Collision collision_if_2_stops = agent_1->would_collide_if_stopped(agent_2, agent_1->stopped, true);

    if (collision_if_1_stops.impending && collision_if_2_stops.impending) {
        // If this case happens, the cars probably are very close, i.e. their current poses already collide.
        // TODO: this situation will create a deadlock where neither car can resume automatically.
        ROS_WARN(
            "Predicted collision between %s and %s can't be prevented by suspending either agent. Both will be "
            "suspended, "
            "resulting in a deadlock. Reset the agents manually.",
            agent_1->name.c_str(), agent_2->name.c_str());
        if (!agent_1->stopped) {
            suspend_agent(agent_1->name, freicar_msgs::AgentCommand::CORRIDOR_CONFLICT);
            agent_1->stopped_by = agent_2->name;
        }

        if (!agent_2->stopped) {
            suspend_agent(agent_2->name, freicar_msgs::AgentCommand::CORRIDOR_CONFLICT);
            agent_2->stopped_by = agent_1->name;
        }

    } else if (!agent_1->stopped && !collision_if_1_stops.impending) {
        ROS_INFO("Will suspend agent %s to prevent the collision with other agent %s.", agent_1->name.c_str(),
                 agent_2->name.c_str());
        suspend_agent(agent_1->name, freicar_msgs::AgentCommand::CORRIDOR_CONFLICT);
        agent_1->stopped_by = agent_2->name;

    } else if (!agent_2->stopped && !collision_if_2_stops.impending) {
        ROS_INFO("Will suspend agent %s to prevent the collision with other agent %s.", agent_2->name.c_str(),
                 agent_1->name.c_str());
        suspend_agent(agent_2->name, freicar_msgs::AgentCommand::CORRIDOR_CONFLICT);
        agent_2->stopped_by = agent_1->name;

    } else {
        // nothing we can do but print a warning. TODO: Not sure if this case can happen, but if it does: debug
        ROS_WARN("Chaperone encountered an unresolvable collision between agents %s (%s) and %s (%s).",
                 agent_1->name.c_str(),
                 agent_1->stopped ? std::string("stopped").c_str() : std::string("not stopped").c_str(),
                 agent_2->name.c_str(),
                 agent_2->stopped ? std::string("stopped").c_str() : std::string("not stopped").c_str());
    }
}

void Chaperone::visualize_collision(const Collision& collision) {
    /*
     * Visualize the collision in rviz by marking the involved footprints with a thicker, red line and increasing the
     * lifetime of the marker to 3s.
     */
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.r = 1.0;

    markers.markers.push_back(agents[collision.agent_1].corridor[collision.collision_index_1].createVisMarker(
        global_frame, static_cast<int>(collision.first_predicted.nsec), color));

    markers.markers.push_back(agents[collision.agent_2].corridor[collision.collision_index_2].createVisMarker(
        global_frame, static_cast<int>(collision.first_predicted.nsec) + 1, color));

    for (auto& m : markers.markers) {
        m.lifetime = ros::Duration(resume_delay);
        m.scale.x = 0.04;
        m.scale.y = 0.04;
        m.scale.z = 0.04;
    }

    marker_pub.publish(markers);
}

void Chaperone::check_and_execute_force_stops() {
    for (auto agent : agents) {
        if (!agent.second.should_force_stop()) continue;

        ROS_INFO_THROTTLE(5.0, "Agent %s has not reacted to the last stop command and will be stopped forcefully.",
                          agent.second.name.c_str());
        agent.second.force_stop();
    }
}

void Chaperone::suspend_agent(const std::string& agent, const uint8_t reason) {
    /* Will suspend the agent with the given ID
     */

    if (agents.find(agent) == agents.end()) {
        ROS_ERROR("Tried to suspend unknown agent %s, this shouldn't happen!", agent.c_str());
        return;
    }

    // mark the agent as stopped
    agents[agent].stopped = true;
    agents[agent].stopped_at = ros::Time::now();

    ROS_INFO("Chaperone will suspend agent %s", agent.c_str());
    send_command(agent, freicar_msgs::AgentCommand::CMD_STOP, reason);
}

void Chaperone::resume_agent(const std::string& agent) {
    /* Will suspend the agent with the given ID
     */

    if (agents.find(agent) == agents.end()) {
        ROS_ERROR("Tried to resume unknown agent %s, this shouldn't happen!", agent.c_str());
        return;
    }

    ROS_INFO("Chaperone will resume agent %s", agent.c_str());
    send_command(agent, freicar_msgs::AgentCommand::CMD_RESUME, freicar_msgs::AgentCommand::OTHER);

    // unset the stopped flags
    agents[agent].stopped = false;
    agents[agent].stopped_by = "";
}

bool Chaperone::handle_track_request(freicar_msgs::TrackRequest& req, freicar_msgs::TrackResponse& resp) {
    /* Adds/removes a agent from the list of tracked agents upon request of the agent.
     */

    // critical section: lock in case multiple track request arrive simultaneously
    agents_mutex.lock();

    if (req.track) {
        // add agent
        ROS_INFO("Agent %s wants to be tracked", req.agent_id.c_str());

        if (agents.find(req.agent_id) != agents.end()) {
            ROS_WARN("Agent %s already registered, chaperone will discard older tracked agent!", req.agent_id.c_str());
        }

        // generate agent object and store it
        TrackedAgent new_agent(nh, &tfl, req.agent_id, req.knows_plan);

        // check if the agent generation was successful
        if (new_agent.valid_tf) {
            agents[new_agent.name] = new_agent;
            agents[new_agent.name].state_sub = nh.subscribe("/" + new_agent.name + "/agent_state", 99,
                                                            &TrackedAgent::update_agent_state, &agents[new_agent.name]);
            agents[new_agent.name].control_mode_sub =
                nh.subscribe("/" + new_agent.name + "/control_report", 99, &TrackedAgent::control_report_callback,
                             &agents[new_agent.name]);
            agents[new_agent.name].control_command_pub =
                nh.advertise<freicar_msgs::ControlCommand>("/" + new_agent.name + "/control", 99);
            agents[new_agent.name].control_mode_pub =
                nh.advertise<std_msgs::Bool>("/" + new_agent.name + "/control_mode", 99);
            resp.success = true;
            ROS_INFO("Successfully registered agent %s", req.agent_id.c_str());
        } else {
            ROS_ERROR("Unable to register agent %s because no initial pose could be found", req.agent_id.c_str());
            resp.success = false;
        }

    } else {
        // delete agent
        ROS_INFO("Agent %s wants to deregister from tracking", req.agent_id.c_str());
        auto i = agents.find(req.agent_id);
        if (i != agents.end()) {
            i->second.state_sub.shutdown();
            i->second.control_mode_sub.shutdown();
            i->second.control_command_pub.shutdown();
            i->second.control_mode_pub.shutdown();
            agents.erase(i);
            resp.success = true;
            ROS_INFO("Successfully de-registered agent %s", req.agent_id.c_str());
        } else {
            ROS_ERROR("Agent %s wants to be removed from chaperone, but doesn't exist", req.agent_id.c_str());
            resp.success = false;
        }
    }

    // free the lock
    agents_mutex.unlock();

    // always return true, since a possible failure of the procedure call is already indicated in resp.success
    // this way, clients can use the return value to tell whether or not the chaperone is online
    return true;
}

void Chaperone::send_command(const std::string& agent_id, const uint8_t cmd, const uint8_t reason) const {
    // assert that command is valid
    assert(cmd == freicar_msgs::AgentCommand::CMD_STOP || cmd == freicar_msgs::AgentCommand::CMD_RESUME);

    // assert that reason is valid
    assert(cmd == freicar_msgs::AgentCommand::CORRIDOR_CONFLICT || cmd == freicar_msgs::AgentCommand::STATIC_OBSTACLE ||
           cmd == freicar_msgs::AgentCommand::OUT_OF_BOUNDS || cmd == freicar_msgs::AgentCommand::OTHER);

    freicar_msgs::AgentCommand cmd_msg;
    cmd_msg.header.stamp = ros::Time::now();

    cmd_msg.authority = "chaperone";
    cmd_msg.target_agent = agent_id;
    cmd_msg.command = cmd;
    cmd_msg.reason = reason;

    cmd_pub.publish(cmd_msg);
}

void Chaperone::visualize_corridors(const std::map<std::string, TrackedAgent>& agents_, ros::Publisher& pub) {
    /*
     * Will send a single MarkerArray message on the given publisher to display all agent's corridors in rviz.
     */
    std_msgs::ColorRGBA vis_color;
    vis_color.r = 1.0;
    vis_color.b = 1.0;
    vis_color.g = 1.0;
    vis_color.a = 0.3;

    visualization_msgs::MarkerArray all_markers;
    for (const auto& agent : agents_) {
        if (!agent.second.valid_corridor) continue;
        visualization_msgs::MarkerArray corridor_markers = agent.second.generate_corridor_markers_msg(vis_color);
        // append
        all_markers.markers.insert(all_markers.markers.end(), corridor_markers.markers.begin(),
                                   corridor_markers.markers.end());
    }
    pub.publish(all_markers);
}
