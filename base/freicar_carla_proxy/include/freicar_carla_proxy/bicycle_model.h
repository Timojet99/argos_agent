#pragma once

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#define L_DIST 0.36505
#define L_R 0.18267

class bicycle_model {
   public:
    bicycle_model();
    void step(const double v, double steering_angle, double time_delta);
    tf2::Vector3 getVelocities();
    double getYawRate();

   private:
    double yaw_rate_;
    double heading_angle_;
    tf2::Vector3 velocities_;
};
