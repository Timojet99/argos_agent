//
// Created by freicar on 12/6/23.
//

#ifndef FREICAR_CHAPERONE_INCLUDE_CHAPERONE_H_
#define FREICAR_CHAPERONE_INCLUDE_CHAPERONE_H_

#include <TrackedAgent.h>
#include <freicar_msgs/AgentCommand.h>
#include <freicar_msgs/Track.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <map>
#include <string>

#include "Collision.h"
#include "GeoFence.h"

class Chaperone {
   public:
    ros::NodeHandle nh;
    tf::TransformListener tfl;
    ros::Publisher cmd_pub;
    ros::ServiceServer track_service;
    ros::Publisher marker_pub;
    ros::Publisher map_boundary_pub;

    // settings
    std::string global_frame;
    float lookahead_duration;
    float sampling_timestep;
    float car_width = 0.4;
    float car_len = 0.6;
    float resume_delay;

    std::mutex agents_mutex;
    std::map<std::string, TrackedAgent> agents;

    // these global pointers will be set in main below

    explicit Chaperone(const ros::NodeHandle& nh_);
    void run();

   private:
    void check_collisions();
    void prevent_collision(const Collision& collision);
    bool handle_track_request(freicar_msgs::TrackRequest& req, freicar_msgs::TrackResponse& resp);
    void send_command(const std::string& agent_id, uint8_t cmd, uint8_t reason) const;
    static void visualize_corridors(const std::map<std::string, TrackedAgent>& agents_, ros::Publisher& pub);
    void suspend_agent(const std::string& agent, uint8_t reason = freicar_msgs::AgentCommand::OTHER);
    void resume_agent(const std::string& agent);
    void visualize_collision(const Collision& collision);
    GeoFence map_boundary;
    void check_boundary();
    void update_all_agents();

    void check_and_execute_force_stops();
};

#endif  // FREICAR_CHAPERONE_INCLUDE_CHAPERONE_H_
