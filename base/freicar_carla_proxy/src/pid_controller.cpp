//
// Created by freicar on 9/7/21.
//

#include "freicar_carla_proxy/pid_controller.h"

float pid_controller::step(const float error, const ros::Time stamp) {
    ros::Duration dt = stamp - prev_t;

    double delta_e = (error - prev_e) / dt.toSec();
    integral += error * dt.toSec();
    integral = std::min(integral, 1.0);

    double out = p_ * error + i_ * integral + d_ * delta_e;

    //        std::cout << "P value:  " << p_ * error << " D value: " << d_ * delta_e << " I value: " << i_ * integral
    //        << std::endl;

    prev_t = stamp;
    prev_e = error;
    return out;
}

void pid_controller::resetIntegral() { integral = 0.0; }
