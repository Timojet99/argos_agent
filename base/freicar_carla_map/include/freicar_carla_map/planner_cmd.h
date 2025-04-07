#pragma once

/*
   this is a shared header to facilitate the use of planners
*/

namespace freicar {
namespace enums {
enum PlannerCommand : unsigned char {
    LEFT = 1,      // go   left   if possible, fail at the junction otherwise
    RIGHT = 2,     // go   right  if possible, fail at the junction otherwise
    STRAIGHT = 3,  // go straight if possible, fail at the junction otherwise
    RANDOM = 4,    // choose a random direction
    POINT = 5,     // lane-star planner to a specific point
    DIRECT = 6,    // direct path planner to a specific point
    EMPTY = 7      // empty plan
};

}  // namespace enums
}  // namespace freicar
