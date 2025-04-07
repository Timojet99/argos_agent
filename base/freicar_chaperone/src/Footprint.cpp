#include <Footprint.h>

#include <algorithm>

Footprint::Footprint() : sim_time(0), sim_distance(0) {}

Footprint::Footprint(std::vector<Eigen::Vector3d> points) : sim_time(0), sim_distance(0) { points_ = points; }

template <typename T>
bool Footprint::OverlapCheck(const T& a0, const T& a1, const T& b0, const T& b1) {
    if (std::min(a0.x(), a1.x()) > std::max(b0.x(), b1.x()) || std::max(a0.x(), a1.x()) < std::min(b0.x(), b1.x()))
        return false;
    if (std::min(a0.y(), a1.y()) > std::max(b0.y(), b1.y()) || std::max(a0.y(), a1.y()) < std::min(b0.y(), b1.y()))
        return false;
    return true;
}

template <typename T, typename P>
bool Footprint::IntersectCheck(const T& p0, const T& p1, const T& p2, const T& p3) {
    if (!OverlapCheck<T>(p0, p1, p2, p3)) return false;
    P s1_x = p1.x() - p0.x();
    P s1_y = p1.y() - p0.y();
    P s2_x = p3.x() - p2.x();
    P s2_y = p3.y() - p2.y();
    P s, t;
    s = (-s1_y * (p0.x() - p2.x()) + s1_x * (p0.y() - p2.y())) / (-s2_x * s1_y + s1_x * s2_y);
    t = (s2_x * (p0.y() - p2.y()) - s2_y * (p0.x() - p2.x())) / (-s2_x * s1_y + s1_x * s2_y);
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) return true;
    return false;
}

bool Footprint::isPointInPoly(const std::vector<Eigen::Vector3d>& poly, const Eigen::Vector3d& point) {
    size_t i, j, c = 0;
    for (i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        if ((((poly[i].y() <= point.y()) && (point.y() < poly[j].y())) ||
             ((poly[j].y() <= point.y()) && (point.y() < poly[i].y()))) &&
            (point.x() <
             (poly[j].x() - poly[i].x()) * (point.y() - poly[i].y()) / (poly[j].y() - poly[i].y()) + poly[i].x()))
            c = !c;
    }
    return static_cast<bool>(c);
}
void Footprint::setPoints(std::vector<Eigen::Vector3d> points) { points_ = points; }

void Footprint::transform(SE3 t) {
    if (!points_.empty()) {
        for (auto& p : points_) {
            p = t * p;
        }
    }
}

bool Footprint::intersect(const Footprint& other) {
    for (size_t im = 0; im < 4; im++) {
        for (size_t io = 0; io < 4; io++) {
            if (IntersectCheck<Eigen::Vector3d, float>(points_[im], points_[(im + 1) % 4], other.points_[io],
                                                       other.points_[(io + 1) % 4])) {
                return true;
            }
        }
    }
    return false;
}

bool Footprint::staticIntersect(const std::vector<Eigen::Vector3d>& other) {
    for (size_t im = 0; im < 4; im++) {
        for (size_t io = 0; io < other.size(); io++) {
            if (IntersectCheck<Eigen::Vector3d, float>(points_[im], points_[(im + 1) % 4], other[io],
                                                       other[(io + 1) % 4])) {
                return true;
            }
        }
    }
    return false;
}

bool Footprint::withinPoly(const std::vector<Eigen::Vector3d>& ply) {
    for (size_t im = 0; im < 4; im++) {
        if (!isPointInPoly(ply, points_[im])) {
            return false;
        }
    }
    return true;
}

const visualization_msgs::Marker Footprint::createVisMarker(std::string frame, int id,
                                                            std_msgs::ColorRGBA color) const {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = frame;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.15);
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.color.r = color.r;
    marker.color.g = color.g;
    marker.color.b = color.b;
    marker.color.a = 1.0;

    for (size_t i = 0; i < points_.size(); i++) {
        geometry_msgs::Point p;
        p.x = points_[i][0];
        p.y = points_[i][1];
        p.z = 0;
        marker.points.push_back(p);
    }
    // add last segment
    geometry_msgs::Point p;
    p.x = points_[0][0];
    p.y = points_[0][1];
    p.z = 0;
    marker.points.push_back(p);

    return marker;
}
