#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/buffer_core.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include "freicar_carla_proxy/sensor_structs.h"
#include "raiscar_msgs/ControlCommand.h"
//#include "freicar_carla_proxy/json.h"
#include <Eigen/Dense>

#include "freicar_carla_proxy/ResetPosition.h"
#include "freicar_carla_proxy/SetPosition.h"
////CARLA STUFF
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorSnapshot.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/client/WorldSnapshot.h>
#include <carla/geom/Transform.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "freicar_carla_proxy/bicycle_model.h"
#include "freicar_carla_proxy/pid_controller.h"

// using json = nlohmann::json;

namespace freicar {
namespace agent {
namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;
namespace type {
static const unsigned int REAL = 0b01;
static const unsigned int SIMULATED = 0b10;
}  // namespace type

class FreiCarCarlaProxy {
   public:
    struct Settings {
        bool comp_mode;
        unsigned int type_code;
        int thread_sleep_ms;
        std::string name;
        std::string owner;
        std::string tf_name;
        std::string sync_topic;
        float height_offset;
        bool spawn_sensors;
        bool use_yaml_spawn;
        float sim_delay;
        cg::Vector3D spawn_point;
        float spawn_heading;
        float brake_multiplier;
        std::string map_path;
        bool sim_sync_mode;
    };
    ~FreiCarCarlaProxy();
    FreiCarCarlaProxy(const FreiCarCarlaProxy&) = delete;
    const FreiCarCarlaProxy& operator=(const FreiCarCarlaProxy&) = delete;
    FreiCarCarlaProxy(FreiCarCarlaProxy::Settings settings, std::shared_ptr<ros::NodeHandle> nh);
    void ActivateCarlaAgent(const std::string& address, unsigned int port, bool spectate);
    void StopAgentThread();
    void SetupCallbacks();

   private:
    void ControlCallback(const raiscar_msgs::ControlCommand::ConstPtr& control_command);
    void Step(unsigned int thread_sleep_ms);
    void SyncCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void SetupSensors();
    void UpdateSimSpectator(cg::Transform target_transform);
    void DestroyCarlaAgent();
    cg::Transform LoadSpawnPoseFromYaml();
    bool ResetPositionCallback(freicar_carla_proxy::ResetPosition::Request& req,
                               freicar_carla_proxy::ResetPosition::Response& res);
    bool SetPositionCallback(freicar_carla_proxy::SetPosition::Request& req,
                             freicar_carla_proxy::SetPosition::Response& res);

    bool TF_Base_Tracker(tf2::Transform to_transform, tf2::Transform& out);

    // agent's identity
    ros::Time last_status_publish_;
    // agent's state
    bool spectate_;
    bool suspended_;
    Settings settings_;
    // publishers & subscribers
    std::shared_ptr<ros::NodeHandle> node_handle_;
    std::unique_ptr<image_transport::ImageTransport> image_transport_;
    ros::Subscriber control_sub_;
    ros::Subscriber sync_sub_;
    ros::Publisher odometry_pub_;
    ros::Publisher odometry_noise_pub_;
    ros::ServiceServer reset_service_server_;
    ros::ServiceServer position_service_server_;
    // thread
    std::thread* agent_thread_;
    bool running_;
    // carla stuff
    std::unique_ptr<cc::Client> carla_client_;
    boost::shared_ptr<carla::client::Vehicle> vehicle_;
    CarlaCamera front_cam_;
    CarlaSemanticCamera semseg_cam_;
    CarlaDepthCamera depth_cam_;
    CarlaLidar lidar_;
    tf2_ros::Buffer tfBuffer_;
    std::unique_ptr<tf2_ros::TransformListener> tfListener_;
    ros::Time send_sync_stamp_;
    bool sync_trigger_rgb;
    bool sync_trigger_depth;
    bool sync_trigger_sem;
    std::default_random_engine generator_;
    cg::Transform initial_pose_;
    ros::Time prev_odo_stamp_;

    nav_msgs::Odometry odom_;
    Eigen::Affine3f odo_pose_ = Eigen::Affine3f::Identity();

    bicycle_model b_model_;
    double current_steering_angle_;

    pid_controller vel_pid_;

    Eigen::Matrix<float, 3, 3> readIntrinsics(const std::string calibration_path);
    cg::Transform LookupCameraExtrinsics(std::string from, std::string to);
    cg::Transform LookupLidarExtrinsics(std::string from, std::string to);

    bool base_tracker_initialized_;
    tf2::Transform T_base_tracker_;
};

namespace {
/* getting rid of ccls warnings */
inline void TypePlaceHolder() {
    (void)type::REAL;
    (void)type::SIMULATED;
}
}  // namespace

}  // namespace agent
}  // namespace freicar
