//
// Created by freicar on 9/6/21.
//

#include "freicar_carla_proxy/bicycle_model.h"

bicycle_model::bicycle_model() {
    velocities_.setX(0.0);
    velocities_.setY(0.0);
    yaw_rate_ = 0.0;
}

void bicycle_model::step(const double v, double steering_angle, double time_delta) {
    heading_angle_ = 0.0;

    double beta = atan(L_R * tan(steering_angle) / L_DIST);
    double x_dot = v * cos(beta + heading_angle_);
    double y_dot = v * sin(beta + heading_angle_);
    double theta_dot = v * tan(steering_angle) * cos(beta) / L_DIST;

    velocities_.setX(x_dot);
    velocities_.setY(y_dot);
    yaw_rate_ = -theta_dot;
}
tf2::Vector3 bicycle_model::getVelocities() { return velocities_; }

double bicycle_model::getYawRate() { return yaw_rate_; }
