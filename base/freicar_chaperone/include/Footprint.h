#ifndef FREICAR_CHAPERONE_INCLUDE_FOOTPRINT_H_
#define FREICAR_CHAPERONE_INCLUDE_FOOTPRINT_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

typedef Eigen::Affine3d SE3;

/* From old chaperone */
class Footprint {
   public:
    explicit Footprint(std::vector<Eigen::Vector3d> points);

    Footprint();

    void transform(SE3 t);

    void setPoints(std::vector<Eigen::Vector3d> points);

    bool intersect(const Footprint& other);

    bool staticIntersect(const std::vector<Eigen::Vector3d>& other);

    bool withinPoly(const std::vector<Eigen::Vector3d>& ply);

    const visualization_msgs::Marker createVisMarker(std::string frame, int id, std_msgs::ColorRGBA color) const;

    float sim_time;
    float sim_distance;
    std::vector<Eigen::Vector3d> points_;
    std::string name;
    tf::Pose pose;

   private:
    template <typename T>
    static bool OverlapCheck(const T& a0, const T& a1, const T& b0, const T& b1);

    template <typename T, typename P>
    bool IntersectCheck(const T& p0, const T& p1, const T& p2, const T& p3);

    bool isPointInPoly(const std::vector<Eigen::Vector3d>& poly, const Eigen::Vector3d& point);
};

#endif  // FREICAR_CHAPERONE_INCLUDE_FOOTPRINT_H_
