//
// Created by freicar on 12/11/23.
//

#ifndef FREICAR_CHAPERONE_INCLUDE_GEOFENCE_H_
#define FREICAR_CHAPERONE_INCLUDE_GEOFENCE_H_

#include <geometry_msgs/PolygonStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class GeoFence {
   public:
    GeoFence();
    explicit GeoFence(const ros::NodeHandle& nh);
    bool inside(const tf::StampedTransform& pose);
    geometry_msgs::PolygonStamped get_marker_msg();
    std::string STOPPING_REASON = "MAP_BOUNDARY";

   private:
    std::string mapping_tool_frame;
    std::string global_frame;
    std::vector<cv::Point2f> fence_polygon;
    bool fence_initialized = false;
    static Eigen::MatrixXd sema_filter(Eigen::MatrixXd& noisy_data, float delta_t, float smoothing_width);
    static std::vector<cv::Point2f> downsample_contour(std::vector<cv::Point2f> contour, double sampling_dist);
    void load_fence_bag(const std::string& bag_filename, float delta_t = 0.06, float smoothing_width = 1.0,
                        float cutoff_start = 0.0, float cutoff_end = 0.0, int max_trajectory_time_s = 300,
                        float sampling_dist = 0.3);
};

#endif  // FREICAR_CHAPERONE_INCLUDE_GEOFENCE_H_
