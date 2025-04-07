//
// Created by freicar on 12/11/23.
//

#include "GeoFence.h"

#include <geometry_msgs/PolygonStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/opencv.hpp>

GeoFence::GeoFence(const ros::NodeHandle& nh) {
    std::string bag_filename;
    nh.param("fence_bag", bag_filename, std::string("./data/fence.bag"));
    nh.param("mapping_tool_frame", mapping_tool_frame, std::string("mapping_tool"));
    nh.param("global_frame", global_frame, std::string("map"));

    load_fence_bag(bag_filename);
}

GeoFence::GeoFence() {
    // empty constructor does nothing
}

bool GeoFence::inside(const tf::StampedTransform& pose) {
    /*
     * Checks if the agent's last pose is within this Geo Fence. DOES NOT check if the agent's last pose is recent!
     */
    if (!fence_initialized) {
        ROS_ERROR("Tried testing if an agent is within map boundary, but no boundary was found!");
        return false;
    }

    cv::Point2d agent_point(pose.getOrigin().x(), pose.getOrigin().y());
    double dist = cv::pointPolygonTest(fence_polygon, agent_point, false);

    // positive dist -> point inside
    return dist > 0;
}

geometry_msgs::PolygonStamped GeoFence::get_marker_msg() {
    /*
     * Will craft a geometry_msgs::PolygonStamped message for visualizing this boundary in Rviz.
     */

    geometry_msgs::PolygonStamped polygon;
    polygon.header.frame_id = global_frame;
    polygon.header.stamp = ros::Time::now();

    if (!fence_initialized) {
        ROS_ERROR("Tried getting Polygon msg from map boundary, but no boundary was found!");
        return polygon;
    }

    for (cv::Point2f p : fence_polygon) {
        geometry_msgs::Point32 p_msg;
        p_msg.x = p.x;
        p_msg.y = p.y;
        p_msg.z = 0.0;
        polygon.polygon.points.push_back(p_msg);
    }
    return polygon;
}

void GeoFence::load_fence_bag(const std::string& bag_filename, float delta_t, float smoothing_width, float cutoff_start,
                              float cutoff_end, int max_trajectory_time_s, float sampling_dist) {
    /* Given a path to a ROS bag, extract the trajectory of the frame with tracker_frame_id in the frame of
     * global_frame_id, then smooth the trajectory. Additionally, trim the trajectory by removing waypoints within a
     * radius of cutoff_start/cutoff_end (in meters) from the beginning and end of the path, respectively.
     * The smoothed and trimmed trajectory is returned as an Nx2 matrix with (x,y) coordinates in the rows.
     */

    // read the file
    rosbag::Bag bag;
    bag.open(bag_filename, rosbag::bagmode::Read);
    std::vector<std::string> topics{"/tf", "/tf_static"};
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // use interpolating transformer with 5 minutes cache time
    tf::Transformer transformer(true, ros::Duration(max_trajectory_time_s));

    // iterate over all messages and store each transform in the transformer object
    for (rosbag::MessageInstance const m : view) {
        tf2_msgs::TFMessage tf_msgs = *m.instantiate<tf2_msgs::TFMessage>();
        for (geometry_msgs::TransformStamped const& tf_msg : tf_msgs.transforms) {
            tf::StampedTransform tf_;
            tf::transformStampedMsgToTF(tf_msg, tf_);
            // feed each Transform message into the transformer
            transformer.setTransform(tf_);
        }
    }

    // use the interpolating transformer to get a vector of tracker coordinates with equal time difference inbetween
    // the samples. This simplifies the moving average filter because we can work with indices instead of timestamps
    // (see paper referenced in method sema_filter).
    // double rec_length = (view.getEndTime() - view.getBeginTime()).toSec();
    // first write the waypoints into an std::vector, then build the Eigen::Matrix we're returning. This is because
    // of a bug where at some time steps in the bag there is no valid transform, which need to be skipped.
    std::vector<tf::Vector3> waypoints_vec;
    // iterate over timepoints in the bag in fixed steps of delta_t seconds
    for (ros::Time t = view.getBeginTime(); t < view.getEndTime(); t += ros::Duration(delta_t)) {
        if (transformer.canTransform(global_frame, mapping_tool_frame, t)) {
            tf::StampedTransform tracker_pose;
            transformer.lookupTransform(global_frame, mapping_tool_frame, t, tracker_pose);
            waypoints_vec.push_back(tracker_pose.getOrigin());
        } else {
            std::cout << "No valid tf at timestep " << t << " , skipping!" << std::endl;
            continue;
        }
    }

    // now that we now the number of waypoints, convert the Vector to an Eigen Matrix
    int n = waypoints_vec.size();
    Eigen::MatrixXd noisy_trajectory;
    noisy_trajectory.resize(n, 2);
    for (int i = 0; i < n; i++) {
        noisy_trajectory(i, 0) = waypoints_vec[i].x();
        noisy_trajectory(i, 1) = waypoints_vec[i].y();
    }

    // Apply filter to smoothen the track
    Eigen::MatrixX2d waypoints = sema_filter(noisy_trajectory, delta_t, smoothing_width);

    int start_index = 1;
    int end_index = n - 2;
    for (; start_index < n; start_index++) {
        if ((waypoints.row(start_index) - waypoints.row(0)).norm() >= cutoff_start) break;
    }
    for (; end_index >= 0; end_index--) {
        if ((waypoints.row(end_index) - waypoints.row(n - 1)).norm() >= cutoff_end) break;
    }
    if (end_index - start_index < 0) {
        throw std::invalid_argument("the trajectory is too short to trim cutoff_start and cutoff_end!");
    }

    // convert to opencv
    std::vector<cv::Point2f> contour;
    for (size_t i = start_index; i < end_index; i++) {
        cv::Point2f point(waypoints.row(i)[0], waypoints.row(i)[1]);
        contour.push_back(point);
    }

    std::vector<cv::Point2f> convex_hull;
    cv::convexHull(contour, convex_hull);

    fence_polygon = downsample_contour(convex_hull, sampling_dist);
    fence_initialized = true;
}

Eigen::MatrixXd GeoFence::sema_filter(Eigen::MatrixXd& noisy_data, float delta_t, float smoothing_width) {
    /* Smooth a time-series of n datapoints (can have multiple dimensions in the rows of the input matrix) using the
     * symmetric exponential moving average (SEMa) filter described in "Estimating Acceleration and Lane-Changing
     * Dynamics from Next Generation Simulation Trajectory Data" (Thiemann et al, 2008,
     * https://www.akesting.de/download/NGSIM_TRR08.pdf). delta_t: the time difference between datapoints
     * smoothing_width: the Delta parameter described in the paper above. If higher, the moving average window will be
     * larger
     */

    int smoothing_width_idx = floor(smoothing_width / delta_t);
    int smoothing_window = 3 * smoothing_width_idx;
    int n = noisy_data.rows();

    Eigen::MatrixXd smooth_data;
    smooth_data.resize(n, noisy_data.cols());
    smooth_data.setZero();
    for (int i = 1; i <= n; i++) {
        // make sure the smoothing window is smaller if we're close to the beginning or end
        int this_smoothing_window = std::min(smoothing_window, std::min(i - 1, n - i));
        double normalizer = 0;
        // compute weighted average of the surrounding points
        for (int k = i - this_smoothing_window; k <= i + this_smoothing_window; k++) {
            double weight = exp(static_cast<double>(-abs(i - k)) / static_cast<double>(smoothing_width_idx));
            smooth_data.row(i - 1) += noisy_data.row(k - 1) * weight;
            normalizer += weight;
        }
        // divide by the sum of weights
        smooth_data.row(i - 1) /= normalizer;
    }
    return smooth_data;
}

std::vector<cv::Point2f> GeoFence::downsample_contour(std::vector<cv::Point2f> contour, double sampling_dist = 0.3) {
    /*
     * Build a new contour by sampling points from the given one, but only include one point every sampling_dist meters.
     */
    assert(!contour.empty());
    std::vector<cv::Point2f> result;
    double dist = 0.0;
    result.push_back(contour.at(0));
    for (size_t i = 1; i < contour.size(); i++) {
        dist += cv::norm(contour.at(i) - contour.at(i - 1));
        if (dist >= sampling_dist) {
            result.push_back(contour.at(i));
            dist = 0.0;
        }
    }
    return result;
}
