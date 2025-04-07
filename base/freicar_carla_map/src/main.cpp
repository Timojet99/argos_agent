#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <cstdio>
#include <ctime>
#include <thread>

#include "freicar_carla_map/WayPoint.h"
#include "freicar_carla_map/planner_cmd.h"
#include "freicar_carla_map/planning/lane_follower.h"
#include "freicar_carla_map/planning/lane_star.h"
#include "freicar_carla_map/thrift_map_proxy.h"
#include "map_core/freicar_carla_map_config.h"
#include "map_core/freicar_carla_map_helper.h"
#define DENSIFICATION_LIMIT 0.22  // meters
static geometry_msgs::Point ToGeometryPoint(const freicar::mapobjects::Point3D& pt) {
    geometry_msgs::Point rt;
    rt.x = pt.x();
    rt.y = pt.y();
    rt.z = pt.z();
    return rt;
}
/* WayPoint service request handler
   Edge cases: if the closest point to current_pos is at index i of lane L
                1) current_pos -> L[i] goes backward, L[i+1] goes forward, L goes on for a couple of steps
                2) current_pos -> L[i] goes backward, L ends at index i
*/
bool HandleWayPointRequest(freicar_carla_map::WayPointRequest& req, freicar_carla_map::WayPointResponse& resp) {
    auto command = static_cast<freicar::enums::PlannerCommand>(req.command);
    auto current_pos =
        freicar::mapobjects::Point3D(req.current_position.x, req.current_position.y, req.current_position.z);
    auto plan = freicar::planning::lane_follower::GetPlan(current_pos, command, req.distance, req.node_count);
    for (size_t i = 0; i < plan.steps.size(); ++i) {
        resp.points.emplace_back(ToGeometryPoint(plan.steps[i].position));
        resp.description.emplace_back(static_cast<unsigned char>(plan.steps[i].path_description));
    }
    return plan.success;
}

/* debugging function to publish plans (either from the lane_star or lane_follower planners) */
void PublishPlan(freicar::planning::Plan& plan, double r, double g, double b, int id, const std::string& name,
                 ros::Publisher& pub) {
    visualization_msgs::MarkerArray list;
    visualization_msgs::Marker* step_number = new visualization_msgs::Marker[plan.size()];
    int num_count = 0;
    visualization_msgs::Marker plan_points;
    plan_points.id = id;
    plan_points.ns = name;
    plan_points.header.stamp = ros::Time();
    plan_points.header.frame_id = "map";
    plan_points.action = visualization_msgs::Marker::ADD;
    plan_points.type = visualization_msgs::Marker::POINTS;
    plan_points.scale.x = 0.03;
    plan_points.scale.y = 0.03;
    plan_points.pose.orientation = geometry_msgs::Quaternion();
    plan_points.color.b = b;
    plan_points.color.a = 0.7;
    plan_points.color.g = g;
    plan_points.color.r = r;
    geometry_msgs::Point p;
    for (size_t i = 0; i < plan.size(); ++i) {
        step_number[i].id = ++num_count + id;
        step_number[i].pose.position.x = p.x = plan[i].position.x();
        step_number[i].pose.position.y = p.y = plan[i].position.y();
        p.z = plan[i].position.z();
        step_number[i].pose.position.z = plan[i].position.z() + 0.1;
        step_number[i].pose.orientation = geometry_msgs::Quaternion();
        step_number[i].ns = name + "_nums";
        step_number[i].header.stamp = ros::Time();
        step_number[i].header.frame_id = "map";
        step_number[i].action = visualization_msgs::Marker::ADD;
        step_number[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        step_number[i].text = std::to_string(i);
        step_number[i].scale.z = 0.055;
        step_number[i].color = plan_points.color;
        list.markers.emplace_back(step_number[i]);
        plan_points.points.emplace_back(p);
    }
    list.markers.emplace_back(plan_points);
    pub.publish(list);
    delete[] step_number;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_framework");
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>();
    ROS_INFO("map framework node started...");
    // starting map proxy
    freicar::map::ThriftMapProxy map_proxy("127.0.0.1", 9091, 9090);
    bool new_map = false;
    // std::string filename = ros::package::getPath("freicar_carla_map") + "/maps/thriftmap_fix.aismap";
    std::string filename;
    if (!ros::param::get("/map_path", filename)) {
        ROS_ERROR("could not find parameter: map_path! map initialization failed.");
        return 0;
    }

    if (!map_proxy.LoadMapFromFile(filename)) {
        ROS_INFO("could not find thriftmap file: %s", filename.c_str());
        map_proxy.StartMapServer();
        // stalling main thread until map is received
        ROS_INFO("waiting for map...");
        while (freicar::map::Map::GetInstance().status() == freicar::map::MapStatus::UNINITIALIZED) {
            ros::Duration(1.0).sleep();
        }
        ROS_INFO("map received");
        new_map = true;
    }
    srand(300);
    // needed some delay before using the map
    ros::Duration(2.0).sleep();
    if (new_map) {
        // thrift creates a file on failed reads
        remove(filename.c_str());
        map_proxy.WriteMapToFile(filename);
        ROS_INFO("saved new map");
    }
    // NOTES:
    // 1) NEVER post process before saving. kinda obvious but still
    // 2) increasing densification limit past a certain point or turning it off
    //    WILL introduce bugs. 22cm seems to be balanced.
    // 	  Possible bug with densification disabled:
    // 		  2 closest lane point to (3.30898, 1.46423, 0) belong to a junction
    // 		  despite obviously belonging to d4c7ecc5-0aa9-49a8-8642-5f8ebb965592
    freicar::map::Map::GetInstance().PostProcess(DENSIFICATION_LIMIT);
    srand(time(NULL));
    // using namespace freicar::planning::lane_star;
    // ros::Publisher tf = node_handle->advertise<visualization_msgs::MarkerArray>("planner_debug", 10, true);
    // using freicar::mapobjects::Point3D;
    // LaneStar planner(100);
    // // auto varz = freicar::map::Map::GetInstance().FindClosestLanePoints(0.0f, 0.0f, 0.0f, 2);
    // auto plan1 = planner.GetPlan(Point3D(1.95487, 3.73705, 0), -3.13996f, Point3D(3.3805, 0.721756, 0), -1.14473f,
    // 0.30); planner.ResetContainers(); auto plan2 = planner.GetPlan(Point3D(3.3805, 0.721756, 0), -2.99003f,
    // Point3D(6.10168, 1.2007, 0), -2.21812f, 0.30);

    // std::thread debug_thread([&]() {
    // 	using namespace std::chrono_literals;
    // 	while (ros::ok()) {
    // 		PublishPlan(plan1, 1.0, 0.1, 0.4, 300, "plan_1", tf);
    // 		PublishPlan(plan2, 0.5, 0.7, 0.1, 300, "plan_2", tf);
    // 		// ROS_INFO("visualized map");
    // 		std::this_thread::sleep_for(1s);
    // 	}
    // });
    ROS_INFO("visualizing map @ 0.1 Hz");
    std::thread map_vis_thread([&]() {
        using namespace std::chrono_literals;
        while (ros::ok()) {
            freicar::map::Map::GetInstance().SendAsRVIZMessage(0.01, 0.01, node_handle);
            // ROS_INFO("visualized map");
            std::this_thread::sleep_for(10s);
        }
    });
    // starting waypoint service
    ros::ServiceServer waypoint_service = node_handle->advertiseService("waypoint_service", HandleWayPointRequest);
    std::cout << "waypoint_service active" << std::endl;
    ros::spin();
    std::cout << "\njoining threads ..." << std::endl;
    map_vis_thread.join();

    return 0;
}
