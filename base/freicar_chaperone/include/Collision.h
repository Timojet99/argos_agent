#ifndef FREICAR_CHAPERONE_INCLUDE_COLLISION_H_
#define FREICAR_CHAPERONE_INCLUDE_COLLISION_H_

#include <ros/ros.h>

#include <string>

struct Collision {
    // Collision is only valid if this is true
    bool impending;

    std::string agent_1;
    std::string agent_2;

    size_t collision_index_1;
    size_t collision_index_2;

    ros::Time first_predicted;
};

#endif  // FREICAR_CHAPERONE_INCLUDE_COLLISION_H_
