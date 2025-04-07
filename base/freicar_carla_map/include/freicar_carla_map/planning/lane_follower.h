#pragma once

#include <string>
#include <vector>

#include "freicar_carla_map/planner_cmd.h"
#include "freicar_carla_map/planning/plan.h"
namespace freicar {
namespace planning {
namespace lane_follower {

freicar::planning::Plan GetPlan(const mapobjects::Point3D& current_position, enums::PlannerCommand command,
                                float distance, unsigned int step_count);
freicar::planning::Plan GetPlan(const std::string& lane_uuid, float req_loffset, enums::PlannerCommand command,
                                float distance, unsigned int step_count);
}  // namespace lane_follower
}  // namespace planning
}  // namespace freicar
