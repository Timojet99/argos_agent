/*
 * FreiCAR class - winter semester 2023/24
 * Do NOT distribute this code to anyone outside the FreiCAR project.
 */

#include <ros/ros.h>

#include "freicar_agent/agent.h"

int main(int argc, char** argv) {
    // AnonymousName option so we can run multiple agents with the same name at once
    ros::init(argc, argv, "freicar_agent");

    Agent agent;
    int success = agent.Run();

    return success;
}
