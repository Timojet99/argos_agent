#include <ros/ros.h>

#include "argos_agent/argos_agent.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "argos_agent");

  ArgosAgent argos_agent;
  int success = argos_agent.Run();

  return success;
}
