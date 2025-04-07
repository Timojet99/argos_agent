#pragma once

#include <ros/ros.h>

class pid_controller {
   public:
    pid_controller(double p, double i, double d) {
        p_ = p;
        i_ = i;
        d_ = d;
        prev_t = ros::Time::now();
        prev_e = 0.0;
        integral = 0.0;
    };

    float step(const float error, const ros::Time stamp);
    void resetIntegral();

   private:
    double p_;
    double i_;
    double d_;
    double integral;
    double prev_e;
    ros::Time prev_t;
};
