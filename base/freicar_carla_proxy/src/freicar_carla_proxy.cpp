#include "freicar_carla_proxy/freicar_carla_proxy.h"

#include <ros/package.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <yaml-cpp/yaml.h>

#include <cmath>

#include "freicar_carla_proxy/agent_config.h"

#define TO_RAD M_PI / 180
#define TO_DEG 180 / M_PI
#define FIXED_FRAME "map"
#define FRONT_CAMERA_FRAME "/zed_camera"
using namespace std::chrono_literals;

namespace freicar {
namespace agent {
FreiCarCarlaProxy::FreiCarCarlaProxy(FreiCarCarlaProxy::Settings settings, std::shared_ptr<ros::NodeHandle> nh)
    : vel_pid_(0.05, 0.15, 0.000) {
    // initialize some stuff & things
    node_handle_ = nh;
    spectate_ = false;
    suspended_ = false;
    base_tracker_initialized_ = false;

    sync_trigger_rgb = false;
    sync_trigger_depth = false;
    sync_trigger_sem = false;

    carla_client_ = nullptr;
    agent_thread_ = nullptr;
    image_transport_ = nullptr;
    send_sync_stamp_ = ros::Time::now();
    prev_odo_stamp_ = ros::Time::now();
    settings_ = settings;
    current_steering_angle_ = 0.0;

    if (settings_.sync_topic != "") {
        bool sync_mode = false;
        if (!ros::param::get("/sim_sync_mode", sync_mode)) {
            ROS_ERROR(
                "ERROR: If any agent has the sync topic turned on, the simulator has to run in SYNCHRONOUS MODE!!");
            std::exit(EXIT_FAILURE);
        }
        if (!sync_mode) {
            ROS_ERROR(
                "ERROR: If any agent has the sync topic turned on, the simulator has to run in SYNCHRONOUS MODE!!");
            std::exit(EXIT_FAILURE);
        }
    }
}

/* destructor */
FreiCarCarlaProxy::~FreiCarCarlaProxy() {
    // releasing thread resources
    StopAgentThread();
    ros::Duration(2.0).sleep();
    DestroyCarlaAgent();
}

bool FreiCarCarlaProxy::TF_Base_Tracker(tf2::Transform to_transform, tf2::Transform& out) {
    while (!base_tracker_initialized_) {
        try {
            geometry_msgs::TransformStamped current_pose;
            current_pose = tfBuffer_.lookupTransform(settings_.tf_name + "/base_link", settings_.tf_name, ros::Time(0));
            tf2::convert(current_pose.transform, T_base_tracker_);
            base_tracker_initialized_ = true;

        } catch (tf2::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    if (base_tracker_initialized_) {
        out = to_transform * T_base_tracker_;
        return true;
    } else {
        return false;
    }
}

/* connects to airsim */
void FreiCarCarlaProxy::ActivateCarlaAgent(const std::string& address, unsigned int port, bool spectate) {
    try {
        carla_client_ = std::make_unique<cc::Client>(address, port);
        carla_client_->SetTimeout(10s);
        std::cout << "connected to CARLA successfully" << std::endl;
        std::cout << "client version: " << carla_client_->GetClientVersion() << '\t'
                  << "server version: " << carla_client_->GetServerVersion() << std::endl;
        auto world = carla_client_->GetWorld();
        auto blueprint_library = world.GetBlueprintLibrary();
        std::cout << "Using 3d model: " << settings_.name << std::endl;
        auto vehicles = blueprint_library->Filter(settings_.name);
        if (vehicles->empty()) {
            ROS_ERROR("ERROR: Did not find car model with name: %s , ... using default model: freicar_10 \n",
                      settings_.name.c_str());
            vehicles = blueprint_library->Filter("freicar_10");
        }
        carla::client::ActorBlueprint blueprint = (*vehicles)[0];
        cg::Transform spawn_pose(cg::Location(settings_.spawn_point), cg::Rotation(0, settings_.spawn_heading, 0));
        if (settings_.use_yaml_spawn) {
            spawn_pose = LoadSpawnPoseFromYaml();
            ros::param::set("~spawn/x", spawn_pose.location.x);
            ros::param::set("~spawn/y", -spawn_pose.location.y);
            ros::param::set("~spawn/z", -spawn_pose.location.z);
            ros::param::set("~spawn/heading", spawn_pose.rotation.yaw);
        }

        initial_pose_ = spawn_pose;
        auto actor = world.TrySpawnActor(blueprint, spawn_pose);
        if (!actor) {
            std::cout << "failed to spawn " << settings_.name << ". exiting..." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        vehicle_ = boost::static_pointer_cast<cc::Vehicle>(actor);
        std::cout << "spawned " << vehicle_->GetDisplayId() << std::endl;
        // turn off physics in mixed case
        if (settings_.type_code == type::SIMULATED)
            vehicle_->SetSimulatePhysics(true);
        else
            vehicle_->SetSimulatePhysics(false);
    } catch (const cc::TimeoutException& e) {
        ROS_ERROR("ERROR: connection to the CARLA server timed out\n%s", e.what());
        std::exit(EXIT_FAILURE);
    } catch (const std::exception& e) {
        ROS_ERROR("ERROR: %s", e.what());
        DestroyCarlaAgent();
        std::exit(EXIT_FAILURE);
    }
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(tfBuffer_);
    if (settings_.spawn_sensors) {
        SetupSensors();
    }
    spectate_ = spectate;
}
/* sets up subscribers, publishers & runs thread */
void FreiCarCarlaProxy::SetupCallbacks() {
    // sim_only handles these topics here
    if (settings_.type_code == type::SIMULATED) {
        odometry_pub_ = node_handle_->advertise<nav_msgs::Odometry>("odometry", 10);
        //        odometry_noise_pub_ = node_handle_->advertise<nav_msgs::Odometry>("odometry_noise", 10);
        control_sub_ = node_handle_->subscribe<raiscar_msgs::ControlCommand>("control", 10,
                                                                             &FreiCarCarlaProxy::ControlCallback, this);
    }
    // start agent thread
    running_ = true;
    if (settings_.sync_topic != "") {
        std::cout << "Syncing to: " << settings_.sync_topic << std::endl;
        sync_sub_ = node_handle_->subscribe<sensor_msgs::CameraInfo>(settings_.sync_topic, 10,
                                                                     &FreiCarCarlaProxy::SyncCallback, this);
    } else {
        std::cout << "Syncing to topic inactive ... " << std::endl;
        agent_thread_ = new std::thread(&FreiCarCarlaProxy::Step, this, settings_.thread_sleep_ms);
    }
    std::cout << settings_.name << ": carla proxy thread started" << std::endl;

    reset_service_server_ = node_handle_->advertiseService(settings_.name + "/reset_position",
                                                           &FreiCarCarlaProxy::ResetPositionCallback, this);
    position_service_server_ =
        node_handle_->advertiseService(settings_.name + "/set_position", &FreiCarCarlaProxy::SetPositionCallback, this);
}

/* thread function for FreiCarCarlaProxy */
void FreiCarCarlaProxy::Step(unsigned int thread_sleep_ms) {
    geometry_msgs::TransformStamped current_pose;
    odom_.header.frame_id = FIXED_FRAME;
    odom_.child_frame_id = settings_.tf_name;
    // sim only
    if (settings_.type_code == type::SIMULATED) {
        current_pose.header.frame_id = FIXED_FRAME;
        current_pose.child_frame_id = settings_.tf_name;
        cg::Transform carla_transform;
        // TF & odom_
        tf2::Quaternion q_carla_rot;
        tf2::Vector3 v_carla_vel;
        tf2::Transform t_carla_rot;
        tf2_ros::TransformBroadcaster tf_broadcaster;

        while (running_) {
            cc::WorldSnapshot wsnap = carla_client_->GetWorld().GetSnapshot();
            boost::optional<cc::ActorSnapshot> asnap = wsnap.Find(vehicle_->GetId());
            carla_transform = asnap->transform;
            // updating/publishing current tf2 transform
            if (settings_.sim_sync_mode) {
                odom_.header.stamp = current_pose.header.stamp = ros::Time(wsnap.GetTimestamp().elapsed_seconds);
            } else {
                odom_.header.stamp = current_pose.header.stamp = ros::Time::now();
            }
            if (odom_.header.stamp <= prev_odo_stamp_) {
                continue;
            }

            double speed = sqrt(odom_.twist.twist.linear.x * odom_.twist.twist.linear.x +
                                odom_.twist.twist.linear.y * odom_.twist.twist.linear.y);

//            std::normal_distribution<float> noise_ang_vel(0.0, 0.2 + (speed * 0.04) / (1000. / thread_sleep_ms));
//            std::normal_distribution<float> noise_x_vel(0.0, 0.1 + (speed * 0.04) / (1000. / thread_sleep_ms));
//            std::normal_distribution<float> noise_y_vel(0.0, 0.1 + (speed * 0.04) / (1000. / thread_sleep_ms));

            std::normal_distribution<float> noise_steering(0.0, 0.05 + (speed * 0.04) / (1000. / thread_sleep_ms));
            std::normal_distribution<float> noise_speed(0.0, (speed * 0.05) / (1000. / thread_sleep_ms));

            current_pose.transform.translation.x = odom_.pose.pose.position.x = carla_transform.location.x;
            current_pose.transform.translation.y = odom_.pose.pose.position.y = -carla_transform.location.y;
            current_pose.transform.translation.z = odom_.pose.pose.position.z = carla_transform.location.z;
            q_carla_rot.setRPY(carla_transform.rotation.roll * TO_RAD, -carla_transform.rotation.pitch * TO_RAD,
                               -carla_transform.rotation.yaw * TO_RAD);
            current_pose.transform.rotation.w = odom_.pose.pose.orientation.w = q_carla_rot.w();
            current_pose.transform.rotation.x = odom_.pose.pose.orientation.x = q_carla_rot.x();
            current_pose.transform.rotation.y = odom_.pose.pose.orientation.y = q_carla_rot.y();
            current_pose.transform.rotation.z = odom_.pose.pose.orientation.z = q_carla_rot.z();

            geometry_msgs::TransformStamped gt_base_link_tf_msg = current_pose;
            gt_base_link_tf_msg.header.frame_id = FIXED_FRAME;
            gt_base_link_tf_msg.child_frame_id = settings_.tf_name + "/base_link_gt";
            tf_broadcaster.sendTransform(gt_base_link_tf_msg);

            tf2::Transform current_tf;
            tf2::convert(current_pose.transform, current_tf);
            if (TF_Base_Tracker(current_tf, current_tf)) {
                current_pose.transform = tf2::toMsg(current_tf);
            }

            if (!settings_.comp_mode) {
                tf_broadcaster.sendTransform(current_pose);
            }

            // converting carla's global odometry to local frame && publishing
            auto angular = asnap->angular_velocity;  // vehicle_->GetAngularVelocity();
            auto velocity = asnap->velocity;         // vehicle_->GetVelocity();
            t_carla_rot.setIdentity();
            t_carla_rot.setRotation(q_carla_rot);
            v_carla_vel.setValue(velocity.x, velocity.y, velocity.z);
            v_carla_vel = t_carla_rot * v_carla_vel;
            odom_.twist.twist.linear.x = v_carla_vel.x();
            odom_.twist.twist.linear.y = v_carla_vel.y();
            odom_.twist.twist.linear.z = v_carla_vel.z();
            odom_.twist.twist.angular.x = angular.x * TO_RAD;
            odom_.twist.twist.angular.y = angular.y * TO_RAD;
            odom_.twist.twist.angular.z = -angular.z * TO_RAD;
            odom_.pose.pose.position.x = 0.0;
            odom_.pose.pose.position.y = 0.0;
            odom_.pose.pose.position.z = 0.0;
            odom_.pose.pose.orientation.w = 1.0;
            odom_.pose.pose.orientation.x = 0.0;
            odom_.pose.pose.orientation.y = 0.0;
            odom_.pose.pose.orientation.z = 0.0;
//            if (false) {  //! settings_.comp_mode
//                odometry_pub_.publish(odom_);
//            } else {
//                // Provide odometry with added gaussian noise
//                odom_.twist.twist.linear.x = odom_.twist.twist.linear.x + noise_x_vel(generator_);
//                odom_.twist.twist.linear.y = odom_.twist.twist.linear.y + noise_y_vel(generator_);
//
//                odom_.twist.twist.angular.x = odom_.twist.twist.angular.x + noise_ang_vel(generator_);
//                odom_.twist.twist.angular.y = odom_.twist.twist.angular.y + noise_ang_vel(generator_);
//                odom_.twist.twist.angular.z = odom_.twist.twist.angular.z + noise_ang_vel(generator_);
//                //                odometry_pub_.publish(odom_); // old noised odometry
//            }

            // Bicycle Model ###########################################################################################
            float time_step = thread_sleep_ms / 1000.f;
            b_model_.step(v_carla_vel.length() + noise_speed(generator_), current_steering_angle_ + noise_steering(generator_), time_step);
            tf2::Vector3 b_vel = b_model_.getVelocities();
            odom_.twist.twist.linear.x = b_vel.getX();
            odom_.twist.twist.linear.y = b_vel.getY();
            odom_.twist.twist.linear.z = 0.0;
            odom_.twist.twist.angular.x = 0.0;
            odom_.twist.twist.angular.y = 0.0;
            odom_.twist.twist.angular.z = b_model_.getYawRate();
            odometry_pub_.publish(odom_);

            // Integrating odometry messages
            float yaw_rate = odom_.twist.twist.angular.z;
            // Static noise on linear velocities
            Eigen::Vector3f trans_rel(odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, 0.0f);

            float x_pos = (trans_rel[0] * time_step);
            float y_pos = (trans_rel[1] * time_step);
            float yaw = (yaw_rate * time_step);

            Eigen::Vector3f pose(x_pos, y_pos, 0);
            pose[0] = x_pos;
            pose[1] = y_pos;
            Eigen::Affine3f T = Eigen::Affine3f::Identity();
            T.translate(pose);
            T.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
            odo_pose_ = odo_pose_.translate(T.translation()).rotate(T.rotation());
            geometry_msgs::TransformStamped odom_pose_msg;
            odom_pose_msg.transform.translation.x = odo_pose_.translation().x();
            odom_pose_msg.transform.translation.y = odo_pose_.translation().y();
            odom_pose_msg.transform.translation.z = odo_pose_.translation().z();

            Eigen::Quaternionf odom_q(odo_pose_.rotation());
            odom_pose_msg.transform.rotation.w = odom_q.w();
            odom_pose_msg.transform.rotation.x = odom_q.x();
            odom_pose_msg.transform.rotation.y = odom_q.y();
            odom_pose_msg.transform.rotation.z = odom_q.z();
            odom_pose_msg.header.stamp = odom_.header.stamp;
            odom_pose_msg.header.frame_id = settings_.tf_name + "/odom_";
            odom_pose_msg.child_frame_id = settings_.tf_name + "/odom_base_link";
            tf_broadcaster.sendTransform(odom_pose_msg);
            prev_odo_stamp_ = odom_.header.stamp;

            std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_ms));
        }
        // mixed case
    } else {
        tf2::Quaternion temp_qtr;
        tf2::Matrix3x3 mat;
        double roll, pitch, yaw;
        while (running_) {
            // updating current transform
            try {
                current_pose = tfBuffer_.lookupTransform(FIXED_FRAME, settings_.tf_name + "/base_link", ros::Time(0));
            } catch (tf2::TransformStorage& ex) {
                ROS_WARN("WARNING: got some buffer error while looking up %s w.r.t. map",
                         (settings_.tf_name + "/base_link").c_str());
                ros::Duration(0.2).sleep();
                continue;
            } catch (tf2::LookupException& ex) {
                ROS_WARN("WARNING: could not find transform %s w.r.t. map", (settings_.tf_name + "/base_link").c_str());
                ros::Duration(0.2).sleep();
                continue;
            } catch (tf2::ExtrapolationException& ex) {
                ROS_WARN("WARNING: got some extrapolation error while looking up %s w.r.t. map",
                         (settings_.tf_name + "/base_link").c_str());
                ros::Duration(0.2).sleep();
                continue;
            }

            temp_qtr.setW(current_pose.transform.rotation.w);
            temp_qtr.setX(current_pose.transform.rotation.x);
            temp_qtr.setY(current_pose.transform.rotation.y);
            temp_qtr.setZ(current_pose.transform.rotation.z);
            mat.setRotation(temp_qtr);
            mat.getEulerYPR(yaw, pitch, roll);
            // sending the transform to carla
            vehicle_->SetTransform(
                cg::Transform(cg::Location(current_pose.transform.translation.x, -current_pose.transform.translation.y,
                                           current_pose.transform.translation.z + settings_.height_offset),
                              cg::Rotation(-pitch * TO_DEG, -yaw * TO_DEG, roll * TO_DEG)));

            std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_ms));
        }
    }
}

/* Computes the extrinsics from a frame to another camera-frame(z-front, x-right) in carlas coordinate system (x-front,
 * y-right) */
cg::Transform FreiCarCarlaProxy::LookupCameraExtrinsics(std::string from, std::string to) {
    geometry_msgs::TransformStamped base_cam_transform;
    while (ros::ok()) {
        try {
            base_cam_transform = tfBuffer_.lookupTransform(settings_.tf_name + from, settings_.tf_name + to,
                                                           ros::Time(0), ros::Duration(3.0));
            break;
        } catch (...) {
            ROS_WARN("could not lookup transform %s w.r.t. %s", (settings_.tf_name + to).c_str(),
                     (settings_.tf_name + from).c_str());
            //            DestroyCarlaAgent();
            //            std::exit(EXIT_FAILURE);
        }
    }
    //        std::cout << "Transform:  " << std::endl;
    //        std::cout << base_cam_transform.transform << std::endl;

    tf2::Stamped<tf2::Transform> base_cam_transform_tf;
    tf2::convert(base_cam_transform, base_cam_transform_tf);

    base_cam_transform_tf.setOrigin(tf2::Vector3(base_cam_transform.transform.translation.x,
                                                 base_cam_transform.transform.translation.y,
                                                 base_cam_transform.transform.translation.z));
    double base_cam_transform_roll, base_cam_transform_pitch, base_cam_transform_yaw;
    tf2::Matrix3x3(base_cam_transform_tf.getRotation())
        .getRPY(base_cam_transform_roll, base_cam_transform_pitch, base_cam_transform_yaw);
    tf2::Vector3 base_cam_transform_t = base_cam_transform_tf.getOrigin();

    // In Carla Y is pointing to the right in the local coordinate (car) coordinate system. In Freicar Y is pointing
    // left.
    base_cam_transform_t.setY(-base_cam_transform_t.getY());
    base_cam_transform_yaw += M_PI / 2;
    base_cam_transform_roll += M_PI / 2;
    //    std::cout << base_cam_transform_tf.getRotation().getX() << " - " << base_cam_transform_tf.getRotation().getY()
    //    << " - " << base_cam_transform_tf.getRotation().getZ() << " - " << base_cam_transform_tf.getRotation().getW()
    //    << " - " << std::endl; std::cout << "Roll: " << base_cam_transform_roll << " Pitch: " <<
    //    base_cam_transform_pitch << " Yaw: " << base_cam_transform_yaw << std::endl;

    // spawn the camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{static_cast<float>(base_cam_transform_t.getX()), static_cast<float>(base_cam_transform_t.getY()),
                     static_cast<float>(base_cam_transform_t.getZ())},
        // In Freicar the transform is given in the camera coordinate system convention (z -front). In Carla it's in the
        // normal local coordinate frame.
        cg::Rotation{static_cast<float>(base_cam_transform_roll * TO_DEG),
                     static_cast<float>((-base_cam_transform_yaw) * TO_DEG),
                     static_cast<float>((base_cam_transform_pitch)*TO_DEG)}};
    //     std::cout << "Pitch: " << static_cast<float>(base_cam_transform_roll * TO_DEG) << std::endl;
    //     std::cout << "Yaw: " << static_cast<float>(base_cam_transform_yaw * TO_DEG) << std::endl;
    //     std::cout << "Roll: " << static_cast<float>(base_cam_transform_pitch * TO_DEG) << std::endl;
    return camera_transform;
}

/* Computes the extrinsics from a frame to another lidar-frame(x-front, y-left) in carlas coordinate system (x-front,
 * y-right) */
cg::Transform FreiCarCarlaProxy::LookupLidarExtrinsics(std::string from, std::string to) {
    geometry_msgs::TransformStamped base_cam_transform =
        tfBuffer_.lookupTransform(settings_.tf_name + from, settings_.tf_name + to, ros::Time(0), ros::Duration(3.0));
    tf2::Stamped<tf2::Transform> base_cam_transform_tf;
    tf2::convert(base_cam_transform, base_cam_transform_tf);
    double base_cam_transform_roll, base_cam_transform_pitch, base_cam_transform_yaw;
    tf2::Matrix3x3(base_cam_transform_tf.getRotation())
        .getRPY(base_cam_transform_roll, base_cam_transform_pitch, base_cam_transform_yaw);
    tf2::Vector3 base_cam_transform_t = base_cam_transform_tf.getOrigin();

    // In Carla Y is pointing to the right in the local coordinate (car) coordinate system. In Freicar Y is pointing
    // left.
    base_cam_transform_t.setY(-base_cam_transform_t.getY());
    base_cam_transform_t.setX(base_cam_transform_t.getX());

    // spawn the camera attached to the vehicle.
    auto camera_transform = cg::Transform{
        cg::Location{static_cast<float>(base_cam_transform_t.getX()), -static_cast<float>(base_cam_transform_t.getY()),
                     static_cast<float>(base_cam_transform_t.getZ())},

        cg::Rotation{static_cast<float>(-base_cam_transform_pitch * TO_DEG),
                     static_cast<float>((base_cam_transform_yaw)*TO_DEG),
                     static_cast<float>((base_cam_transform_roll)*TO_DEG)}};

    return camera_transform;
}

bool FreiCarCarlaProxy::ResetPositionCallback(freicar_carla_proxy::ResetPosition::Request& req,
                                              freicar_carla_proxy::ResetPosition::Response& res) {
    vehicle_->SetTransform(initial_pose_);
    res.success = true;
    return true;
}

bool FreiCarCarlaProxy::SetPositionCallback(freicar_carla_proxy::SetPosition::Request& req,
                                            freicar_carla_proxy::SetPosition::Response& res) {
    cg::Transform new_pose(cg::Location(req.x, -req.y, req.z), cg::Rotation(0, req.heading, 0));
    vehicle_->SetTransform(new_pose);
    res.success = true;
    return true;
}

/* control callback function */
void FreiCarCarlaProxy::ControlCallback(const raiscar_msgs::ControlCommand::ConstPtr& control_command) {
    // Apply control to vehicle.
    static cc::Vehicle::Control control;
    control.manual_gear_shift = false;
    if (!suspended_) {
        control.hand_brake = control_command->hand_brake;
        if (control_command->throttle_mode == 0) {
            control.throttle = control_command->throttle;
            vel_pid_.resetIntegral();
        } else {
            double pid_vel_out = 0.;
            double current_v = odom_.twist.twist.linear.x;
            if (abs(control_command->throttle) >= 0) {
                pid_vel_out = vel_pid_.step(control_command->throttle - current_v, ros::Time::now());
            } else {
                pid_vel_out = 0.0;
                vel_pid_.resetIntegral();
            }

            if (control_command->throttle == 0.0) {
                control.brake = 1.0;
                control.hand_brake = true;
            } else {
                control.brake = 0.0;
                control.hand_brake = false;
            }

            control.throttle = std::max(std::min(pid_vel_out, 1.0), -1.0);
        }
        control.brake = control_command->brake;
        if (control_command->throttle >= 0.0f) {
            control.reverse = false;
        } else {
            control.reverse = true;
        }
        if (control_command->hand_brake) {
            control.throttle = 0.0;
        }
        // For carla the input range of the steering command is -1.0 <-> 1.0
        double a = -10.949;
        double b = 50.922;
        control.steer = (std::sqrt(4 * a * abs(control_command->steering) + b * b) - b) / (2. * a);
        if (control_command->steering < 0) {
            control.steer = -control.steer;
        }
        current_steering_angle_ = -control_command->steering * (M_PI / 180.);
    } else {
        control.throttle = 0.0f;
        control.hand_brake = true;
    }
    control.steer = -control.steer;
    vehicle_->ApplyControl(control);
}

/* sync msg topic callback */
void FreiCarCarlaProxy::SyncCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    geometry_msgs::TransformStamped current_pose;
    tf2::Quaternion temp_qtr;
    tf2::Matrix3x3 mat;
    double roll, pitch, yaw;

    if (!(msg->header.stamp > send_sync_stamp_)) {
        return;
    }
    send_sync_stamp_ = msg->header.stamp;

    if (sync_trigger_rgb || sync_trigger_sem || sync_trigger_depth) {
        std::cout << "Warning: New sync pulse arrived before getting the simulated image..." << std::endl;
    }

    sync_trigger_rgb = true;
    sync_trigger_sem = true;
    sync_trigger_depth = true;

    if (running_) {
        // updating current transform
        try {
            current_pose = tfBuffer_.lookupTransform(FIXED_FRAME, settings_.tf_name + "/base_link", msg->header.stamp);
        } catch (tf2::TransformStorage& ex) {
            ROS_WARN("WARNING: got some buffer error while looking up %s w.r.t. map",
                     (settings_.tf_name + "/base_link").c_str());
            ros::Duration(0.2).sleep();
        } catch (tf2::LookupException& ex) {
            ROS_WARN("WARNING: could not find transform %s w.r.t. map", (settings_.tf_name + "/base_link").c_str());
            ros::Duration(0.2).sleep();
        } catch (tf2::ExtrapolationException& ex) {
            ROS_WARN("WARNING: got some extrapolation error while looking up %s w.r.t. map",
                     (settings_.tf_name + "/base_link").c_str());
            ros::Duration(0.2).sleep();
        }
        temp_qtr.setW(current_pose.transform.rotation.w);
        temp_qtr.setX(current_pose.transform.rotation.x);
        temp_qtr.setY(current_pose.transform.rotation.y);
        temp_qtr.setZ(current_pose.transform.rotation.z);
        mat.setRotation(temp_qtr);
        mat.getEulerYPR(yaw, pitch, roll);
        // sending the transform to carla
        vehicle_->SetTransform(
            cg::Transform(cg::Location(current_pose.transform.translation.x, -current_pose.transform.translation.y,
                                       current_pose.transform.translation.z + settings_.height_offset),
                          cg::Rotation(-pitch * TO_DEG, -yaw * TO_DEG, roll * TO_DEG)));
        carla_client_->GetWorld().Tick(10s);
        // std::cout << "Tick" << std::endl;
    }
}

int roundUp(int numToRound, int multiple) {
    if (multiple == 0) return numToRound;

    int remainder = numToRound % multiple;
    if (remainder == 0) return numToRound;

    return numToRound + multiple - remainder;
}

cv::Mat translateImg(cv::Mat& img, double offsetx, double offsety, int flags) {
    cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img, img, trans_mat, img.size(), flags);
    return img;
}

float acot(float value) { return (M_PI / 2 - atan(value)); }

cg::Transform FreiCarCarlaProxy::LoadSpawnPoseFromYaml() {
    std::string map_name = std::filesystem::path(settings_.map_path).stem();
    std::string yaml_path = ros::package::getPath("freicar_launch") + "/spawn_positions/" + map_name + "_spawn.yaml";
    YAML::Node base = YAML::LoadFile(yaml_path);
    YAML::Node pose_node = base["spawn_pose"];
    const float x = pose_node["x"].as<float>();
    const float y = -pose_node["y"].as<float>();
    const float z = pose_node["z"].as<float>();
    const float heading = pose_node["heading"].as<float>();
    return cg::Transform(cg::Location(x, y, z), cg::Rotation(0, heading, 0));
}

/* initializes the structures for the camera, lidar & depth measurements */
void FreiCarCarlaProxy::SetupSensors() {
    image_transport_ = std::make_unique<image_transport::ImageTransport>(*node_handle_);

    std::string yaml_path;
    if (!ros::param::get("~sensor_description_file", yaml_path)) {
        return;
    }
    std::cout << yaml_path << std::endl;
    auto bp_library = vehicle_->GetWorld().GetBlueprintLibrary();
    YAML::Node base = YAML::LoadFile(yaml_path);
    YAML::Node front_cam_node = base["camera-front"];
    if (front_cam_node.IsDefined()) {
        std::cout << "setting up front camera" << std::endl;
        front_cam_.rgb_publisher = image_transport_->advertise("/sim/camera/rgb/front/image", 10);
        front_cam_.info_publisher =
            node_handle_->advertise<sensor_msgs::CameraInfo>("/sim/camera/rgb/front/image/camera_info", 10);
        // usual camera info stuff
        front_cam_.cam_info.header.frame_id = settings_.name + "_sim_rgb_front_info";
        front_cam_.header.frame_id = settings_.name + "_sim_rgb_front";
        front_cam_.cam_info.header.seq = front_cam_.header.seq = 0;

        auto cam_blueprint = *bp_library->Find(front_cam_node["type"].as<std::string>());
        // setting camera attributes from the yaml
        const float cx = front_cam_node["cx"].as<float>();
        const float cy = front_cam_node["cy"].as<float>();
        const int sx = front_cam_node["image_size_x"].as<int>();
        const int sy = front_cam_node["image_size_y"].as<int>();
        const float shift_x = cx - (sx / 2.);
        const float shift_y = cy - (sy / 2.);
        const int margin_x = roundUp(std::abs(int(shift_x)), 16);
        const int margin_y = roundUp(std::abs(int(shift_y)), 16);

        for (YAML::const_iterator it = front_cam_node.begin(); it != front_cam_node.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (cam_blueprint.ContainsAttribute(key)) cam_blueprint.SetAttribute(key, it->second.as<std::string>());
        }
        // spawn the camera attached to the vehicle.]
        float size_x = sx + margin_x * 2;
        float size_y = sy + margin_y * 2;
        cam_blueprint.SetAttribute("image_size_x", std::to_string(size_x));
        cam_blueprint.SetAttribute("image_size_y", std::to_string(size_y));
        // recalculate fov based on image size and focal length
        const float fx = front_cam_node["fx"].as<float>();
        float fov = (360 * (acot(fx / (size_x / 2.)))) / M_PI;
        cam_blueprint.SetAttribute("fov", std::to_string(fov));

        front_cam_.margin_x = margin_x;
        front_cam_.margin_y = margin_y;
        front_cam_.shift_x = shift_x;
        front_cam_.shift_y = shift_y;
        // Calculate margin for principal point cropping
        cg::Transform camera_transform = LookupCameraExtrinsics("/base_link", FRONT_CAMERA_FRAME);
        auto generic_actor = vehicle_->GetWorld().SpawnActor(cam_blueprint, camera_transform, vehicle_.get());
        front_cam_.sensor = boost::static_pointer_cast<cc::Sensor>(generic_actor);
        // register a callback to publish images
        front_cam_.sensor->Listen([this](boost::shared_ptr<carla::sensor::SensorData> data) {
            if (settings_.sync_topic != "") {
                if (sync_trigger_rgb) {
                    front_cam_.cam_info.header.stamp = front_cam_.header.stamp = send_sync_stamp_;
                    sync_trigger_rgb = false;
                } else {
                    return;
                }
            } else {
                if (settings_.type_code == type::REAL) {
                    front_cam_.cam_info.header.stamp = front_cam_.header.stamp =
                        ros::Time(data->GetTimestamp()) - ros::Duration(settings_.sim_delay);
                } else {
                    if (settings_.sim_sync_mode) {
                        front_cam_.cam_info.header.stamp = front_cam_.header.stamp = ros::Time(data->GetTimestamp());
                    } else {
                        front_cam_.cam_info.header.stamp = front_cam_.header.stamp = ros::Time::now();
                    }
                }
            }

            auto image = boost::static_pointer_cast<csd::Image>(data);
            auto image_view = carla::image::ImageView::MakeView(*image);
            auto rgb_view = boost::gil::color_converted_view<boost::gil::rgb8_pixel_t>(image_view);
            typedef decltype(rgb_view)::value_type pixel;
            static_assert(sizeof(pixel) == 3, "R, G & B");
            pixel raw_data[rgb_view.width() * rgb_view.height()];
            boost::gil::copy_pixels(rgb_view, boost::gil::interleaved_view(rgb_view.width(), rgb_view.height(),
                                                                           raw_data, rgb_view.width() * sizeof(pixel)));
            front_cam_.carla_image = cv::Mat(rgb_view.height(), rgb_view.width(), CV_8UC3, raw_data);
            front_cam_.carla_image =
                translateImg(front_cam_.carla_image, (front_cam_.shift_x), (front_cam_.shift_y), CV_INTER_LINEAR);
            const cv::Rect roi(front_cam_.margin_x, front_cam_.margin_y,
                               int(front_cam_.carla_image.cols - (front_cam_.margin_x * 2)),
                               int(front_cam_.carla_image.rows - (front_cam_.margin_y * 2)));
            front_cam_.carla_image = front_cam_.carla_image(roi);

            front_cam_.cam_info.width = front_cam_.carla_image.cols;
            front_cam_.cam_info.height = front_cam_.carla_image.rows;
            sensor_msgs::ImagePtr rgb_msg =
                cv_bridge::CvImage(front_cam_.header, "rgb8", front_cam_.carla_image).toImageMsg();
            front_cam_.rgb_publisher.publish(rgb_msg);
            front_cam_.info_publisher.publish(front_cam_.cam_info);
        });
    }
    // set up lidar
    YAML::Node lidar_node = base["lidar"];
    if (lidar_node.IsDefined()) {
        std::cout << "setting up lidar" << std::endl;
        auto lidar_blueprint = *bp_library->Find(lidar_node["type"].as<std::string>());
        // setting lidar attributes from the yaml
        for (YAML::const_iterator it = lidar_node.begin(); it != lidar_node.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (lidar_blueprint.ContainsAttribute(key)) lidar_blueprint.SetAttribute(key, it->second.as<std::string>());
        }
        // spawn the lidar sensor attached to the vehicle.
        cg::Transform lidar_transform = LookupLidarExtrinsics("/base_link", "/lidar_link");
        auto lidar_actor = vehicle_->GetWorld().SpawnActor(lidar_blueprint, lidar_transform, vehicle_.get());
        lidar_.sensor = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
        lidar_.publisher = node_handle_->advertise<CarlaRGBPointCloud>("/sim/lidar", 20);
        // register a callback to publish point cloud
        lidar_.sensor->Listen([this](boost::shared_ptr<carla::sensor::SensorData> data) {
            auto pointcloud = boost::static_pointer_cast<csd::LidarMeasurement>(data);
            auto pc_size = pointcloud->size();
            CarlaRGBPointCloud pcl_message;
            pcl_message.header.frame_id = settings_.tf_name + "/lidar_link";
            pcl_message.height = pointcloud->GetChannelCount();
            pcl_message.width = pc_size / pcl_message.height;
            pcl_message.points.reserve(pc_size);
            pcl::PointXYZRGB point;
            for (size_t i = 0; i < pc_size; ++i) {
                float fr = static_cast<float>(i) / static_cast<float>(pc_size);
                point.r = fr * 255;
                point.g = 255 - fr * 255;
                point.b = 18 + fr * 20;
                point.x = -pointcloud->at(i).y;
                point.y = -pointcloud->at(i).x;
                point.z = -pointcloud->at(i).z;
                pcl_message.points.push_back(point);
            }
            ros::Time stamp_lidar;
            if (settings_.type_code == type::REAL) {
                stamp_lidar = ros::Time(data->GetTimestamp()) - ros::Duration(settings_.sim_delay);
            } else {
                if (settings_.sim_sync_mode) {
                    stamp_lidar = ros::Time(data->GetTimestamp());
                } else {
                    stamp_lidar = ros::Time::now();
                }
            }
            try {
                pcl_conversions::toPCL(stamp_lidar, pcl_message.header.stamp);
                lidar_.publisher.publish(pcl_message);
                pcl_message.points.clear();
            } catch (const std::exception& e) {
                std::cout << "Could not send pcl message " << std::endl;
            }
        });
    }
    YAML::Node depth_cam_node = base["camera-depth"];
    if (depth_cam_node.IsDefined()) {
        std::cout << "setting up depth camera" << std::endl;
        depth_cam_.rgb_publisher = image_transport_->advertise("/sim/camera/depth/front/image", 10);
        depth_cam_.float_publisher = image_transport_->advertise("/sim/camera/depth/front/image_float", 10);
        depth_cam_.info_publisher =
            node_handle_->advertise<sensor_msgs::CameraInfo>("/sim/camera/depth/front/image/camera_info", 10);
        // usual camera info stuffw
        depth_cam_.cam_info.header.frame_id = settings_.name + "_sim_depth_front_info";
        depth_cam_.header.frame_id = settings_.name + "_sim_depth_front";
        depth_cam_.cam_info.header.seq = depth_cam_.header.seq = 0;

        auto cam_blueprint = *bp_library->Find(depth_cam_node["type"].as<std::string>());
        // setting camera attributes from the yaml
        const float cx = depth_cam_node["cx"].as<float>();
        const float cy = depth_cam_node["cy"].as<float>();
        const int sx = depth_cam_node["image_size_x"].as<int>();
        const int sy = depth_cam_node["image_size_y"].as<int>();
        const float shift_x = cx - (sx / 2.);
        const float shift_y = cy - (sy / 2.);
        //        std::cout << "Adapt to " << shift_x << " , " << shift_y << std::endl;
        const int margin_x = roundUp(std::abs(int(shift_x)), 16);
        const int margin_y = roundUp(std::abs(int(shift_y)), 16);
        for (YAML::const_iterator it = depth_cam_node.begin(); it != depth_cam_node.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (cam_blueprint.ContainsAttribute(key)) cam_blueprint.SetAttribute(key, it->second.as<std::string>());
        }

        float size_x = sx + margin_x * 2;
        float size_y = sy + margin_y * 2;
        cam_blueprint.SetAttribute("image_size_x", std::to_string(size_x));
        cam_blueprint.SetAttribute("image_size_y", std::to_string(size_y));
        // recalculate fov based on image size and focal length
        const float fx = depth_cam_node["fx"].as<float>();
        float fov = (360 * (acot(fx / (size_x / 2.)))) / M_PI;
        cam_blueprint.SetAttribute("fov", std::to_string(fov));

        cam_blueprint.SetAttribute("image_size_x", std::to_string(sx + margin_x * 2));
        cam_blueprint.SetAttribute("image_size_y", std::to_string(sy + margin_y * 2));
        depth_cam_.margin_x = margin_x;
        depth_cam_.margin_y = margin_y;
        depth_cam_.shift_x = shift_x;
        depth_cam_.shift_y = shift_y;

        // spawn the depth camera attached to the vehicle.
        cg::Transform camera_transform = LookupCameraExtrinsics("/base_link", FRONT_CAMERA_FRAME);

        auto generic_actor = vehicle_->GetWorld().SpawnActor(cam_blueprint, camera_transform, vehicle_.get());
        depth_cam_.sensor = boost::static_pointer_cast<cc::Sensor>(generic_actor);
        // register a callback to publish images
        depth_cam_.sensor->Listen([this](boost::shared_ptr<carla::sensor::SensorData> data) {
            if (settings_.sync_topic != "") {
                if (sync_trigger_depth) {
                    depth_cam_.cam_info.header.stamp = depth_cam_.header.stamp = send_sync_stamp_;
                    sync_trigger_sem = false;
                } else {
                    return;
                }
            } else {
                if (settings_.type_code == type::REAL) {
                    depth_cam_.cam_info.header.stamp = depth_cam_.header.stamp =
                        ros::Time(data->GetTimestamp()) - ros::Duration(settings_.sim_delay);
                } else {
                    if (settings_.sim_sync_mode) {
                        depth_cam_.cam_info.header.stamp = depth_cam_.header.stamp = ros::Time(data->GetTimestamp());
                    } else {
                        depth_cam_.cam_info.header.stamp = depth_cam_.header.stamp = ros::Time::now();
                    }
                }
            }
            auto image = boost::static_pointer_cast<csd::Image>(data);
            cv::Mat cvm(image->GetHeight(), image->GetWidth(), CV_8UC4, image->data());
            // Simulate principal point
            cvm = translateImg(cvm, (depth_cam_.shift_x), (depth_cam_.shift_y), CV_INTER_NN);
            const cv::Rect roi(depth_cam_.margin_x, depth_cam_.margin_y, int(cvm.cols - (depth_cam_.margin_x * 2)),
                               int(cvm.rows - (depth_cam_.margin_y * 2)));
            cvm = cvm(roi);
            std::vector<cv::Mat> splits;
            cv::split(cvm, splits);
            cv::Mat cvm_f(image->GetHeight(), image->GetWidth(), CV_32FC1);
            cv::Mat R, G, B;
            splits[2].convertTo(R, CV_32FC1);
            splits[1].convertTo(G, CV_32FC1);
            splits[0].convertTo(B, CV_32FC1);
            cvm_f = ((R + G * 256.0 + B * (256.0 * 256.0)) / (256.0 * 256.0 * 256.0 - 1.0)) * 1000.0;
            // auto image_view =
            // carla::image::ImageView::MakeColorConvertedView(carla::image::ImageView::MakeView(*image),
            // 																  carla::image::ColorConverter::LogarithmicDepth());
            // // auto image_view = carla::image::ImageView::MakeView(*image);
            // auto grayscale_view = boost::gil::color_converted_view<boost::gil::gray8_pixel_t>(image_view);
            // typedef decltype(grayscale_view)::value_type pixel;
            // static_assert(sizeof(pixel) == 1, "single channel");
            // pixel raw_data[grayscale_view.width() * grayscale_view.height()];
            // boost::gil::copy_pixels(grayscale_view, boost::gil::interleaved_view(grayscale_view.width(),
            // 																	 grayscale_view.height(),
            // 																	 raw_data,
            // 																	 grayscale_view.width()
            // * sizeof(pixel))); depth_cam_.carla_image = cv::Mat(grayscale_view.height(), grayscale_view.width(),
            // CV_8UC1, raw_data); sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(depth_cam_.header, "mono8",
            // depth_cam_.carla_image).toImageMsg(); depth_cam_.rgb_publisher.publish(rgb_msg);
            depth_cam_.cam_info.width = image->GetWidth();
            depth_cam_.cam_info.height = image->GetHeight();
            depth_cam_.float_publisher.publish(cv_bridge::CvImage(depth_cam_.header, "32FC1", cvm_f).toImageMsg());
            depth_cam_.info_publisher.publish(depth_cam_.cam_info);
        });
    }
    YAML::Node semseg_cam_node = base["camera-semseg"];
    if (semseg_cam_node.IsDefined()) {
        std::cout << "setting up semseg camera" << std::endl;
        semseg_cam_.rgb_publisher = image_transport_->advertise("/sim/camera/semseg/front/image", 10);
        semseg_cam_.raw_publisher = image_transport_->advertise("/sim/camera/semseg/front/image_raw", 10);
        semseg_cam_.info_publisher =
            node_handle_->advertise<sensor_msgs::CameraInfo>("/sim/camera/semseg/front/image/camera_info", 10);
        // usual camera info stuff
        semseg_cam_.cam_info.header.frame_id = settings_.name + "_sim_semseg_front_info";
        semseg_cam_.header.frame_id = settings_.name + "_sim_semseg_front";
        semseg_cam_.cam_info.header.seq = semseg_cam_.header.seq = 0;

        auto cam_blueprint = *bp_library->Find(semseg_cam_node["type"].as<std::string>());
        // setting camera attributes from the yaml
        const float cx = semseg_cam_node["cx"].as<float>();
        const float cy = semseg_cam_node["cy"].as<float>();
        const int sx = semseg_cam_node["image_size_x"].as<int>();
        const int sy = semseg_cam_node["image_size_y"].as<int>();
        const float shift_x = cx - (sx / 2.);
        const float shift_y = cy - (sy / 2.);
        const int margin_x = roundUp(std::abs(int(shift_x)), 16);
        const int margin_y = roundUp(std::abs(int(shift_y)), 16);

        for (YAML::const_iterator it = semseg_cam_node.begin(); it != semseg_cam_node.end(); ++it) {
            auto key = it->first.as<std::string>();
            if (cam_blueprint.ContainsAttribute(key)) cam_blueprint.SetAttribute(key, it->second.as<std::string>());
        }
        // spawn the camera attached to the vehicle.

        float size_x = sx + margin_x * 2;
        float size_y = sy + margin_y * 2;
        cam_blueprint.SetAttribute("image_size_x", std::to_string(size_x));
        cam_blueprint.SetAttribute("image_size_y", std::to_string(size_y));
        // recalculate fov based on image size and focal length
        const float fx = semseg_cam_node["fx"].as<float>();
        float fov = (360 * (acot(fx / (size_x / 2.)))) / M_PI;
        cam_blueprint.SetAttribute("fov", std::to_string(fov));

        semseg_cam_.margin_x = margin_x;
        semseg_cam_.margin_y = margin_y;
        semseg_cam_.shift_x = shift_x;
        semseg_cam_.shift_y = shift_y;
        cg::Transform camera_transform = LookupCameraExtrinsics("/base_link", FRONT_CAMERA_FRAME);
        auto generic_actor = vehicle_->GetWorld().SpawnActor(cam_blueprint, camera_transform, vehicle_.get());
        semseg_cam_.sensor = boost::static_pointer_cast<cc::Sensor>(generic_actor);
        // register a callback to save images to disk.
        semseg_cam_.sensor->Listen([this](boost::shared_ptr<carla::sensor::SensorData> data) {
            if (settings_.sync_topic != "") {
                if (sync_trigger_sem) {
                    semseg_cam_.cam_info.header.stamp = semseg_cam_.header.stamp = send_sync_stamp_;
                    sync_trigger_sem = false;
                } else {
                    return;
                }
            } else {
                if (settings_.type_code == type::REAL) {
                    semseg_cam_.cam_info.header.stamp = semseg_cam_.header.stamp =
                        ros::Time(data->GetTimestamp()) - ros::Duration(settings_.sim_delay);
                } else {
                    if (settings_.sim_sync_mode) {
                        semseg_cam_.cam_info.header.stamp = semseg_cam_.header.stamp = ros::Time(data->GetTimestamp());
                    } else {
                        semseg_cam_.cam_info.header.stamp = semseg_cam_.header.stamp = ros::Time::now();
                    }
                }
            }
            auto image = boost::static_pointer_cast<csd::Image>(data);
            cv::Mat cvm(image->GetHeight(), image->GetWidth(), CV_8UC4, image->data());
            cvm = translateImg(cvm, (semseg_cam_.shift_x), (semseg_cam_.shift_y), CV_INTER_NN);
            const cv::Rect roi(semseg_cam_.margin_x, semseg_cam_.margin_y, int(cvm.cols - (semseg_cam_.margin_x * 2)),
                               int(cvm.rows - (semseg_cam_.margin_y * 2)));
            cvm = cvm(roi);
            std::vector<cv::Mat> splits;
            cv::split(cvm, splits);
            // auto image_view =
            // carla::image::ImageView::MakeColorConvertedView(carla::image::ImageView::MakeView(*image),
            // 																  carla::image::ColorConverter::CityScapesPalette());
            // auto rgb_view = boost::gil::color_converted_view<boost::gil::rgb8_pixel_t>(image_view);
            // typedef decltype(rgb_view)::value_type pixel;
            // static_assert(sizeof(pixel) == 3, "R, G & B");
            // pixel raw_data[rgb_view.width() * rgb_view.height()];
            // boost::gil::copy_pixels(rgb_view, boost::gil::interleaved_view(rgb_view.width(),
            // 															   rgb_view.height(),
            // 															   raw_data,
            // 															   rgb_view.width()
            // * sizeof(pixel))); semseg_cam_.carla_image = cv::Mat(rgb_view.height(), rgb_view.width(), CV_8UC3,
            // raw_data); sensor_msgs::ImagePtr rgb_msg = cv_bridge::CvImage(semseg_cam_.header, "rgb8",
            // semseg_cam_.carla_image).toImageMsg(); semseg_cam_.rgb_publisher.publish(rgb_msg);
            semseg_cam_.cam_info.width = semseg_cam_.carla_image.cols;
            semseg_cam_.cam_info.height = semseg_cam_.carla_image.rows;
            if (!settings_.comp_mode) {
                semseg_cam_.info_publisher.publish(semseg_cam_.cam_info);
                semseg_cam_.raw_publisher.publish(
                    cv_bridge::CvImage(semseg_cam_.header, "8UC1", splits[2]).toImageMsg());
            }
        });
    }
}

/* stops the agent thread */
void FreiCarCarlaProxy::StopAgentThread() {
    running_ = false;
    if (agent_thread_ && agent_thread_->joinable()) {
        agent_thread_->join();
        delete agent_thread_;
    }
}

/* destroys all the agents in the simulation & their sensors. usually called before the program
   is about to exit with a ROS_ERROR */
void FreiCarCarlaProxy::DestroyCarlaAgent() {
    if (front_cam_.sensor) {
        front_cam_.sensor->Stop();
        front_cam_.sensor->Destroy();
        front_cam_.sensor = nullptr;
    }
    if (depth_cam_.sensor) {
        depth_cam_.sensor->Stop();
        depth_cam_.sensor->Destroy();
        depth_cam_.sensor = nullptr;
    }
    if (semseg_cam_.sensor) {
        semseg_cam_.sensor->Stop();
        semseg_cam_.sensor->Destroy();
        semseg_cam_.sensor = nullptr;
    }
    if (lidar_.sensor) {
        lidar_.sensor->Stop();
        lidar_.sensor->Destroy();
        lidar_.sensor = nullptr;
    }
    if (vehicle_) {
        vehicle_->Destroy();
        vehicle_ = nullptr;
    }
    if (carla_client_) {
        carla_client_.reset();
    }
    if (image_transport_) image_transport_.reset();
}

/* updates the spectator view in the simulator. this is a debug feature but could be useful */
void FreiCarCarlaProxy::UpdateSimSpectator(cg::Transform target_transform) {
    static cg::Transform spec_transform;
    spec_transform.rotation.pitch = -20.0f;
    target_transform.location -= 2.0f * target_transform.GetForwardVector();
    target_transform.location.z += 1.0f;
    spec_transform.location = config::kSpectatorCamSmoothFactor * spec_transform.location +
                              (1 - config::kSpectatorCamSmoothFactor) * target_transform.location;
    // angle correction (jumps between 180 <-> -180)
    auto ang_diff = spec_transform.rotation.yaw - target_transform.rotation.yaw;
    if (std::abs(ang_diff) > 200.0f) {
        ang_diff = (ang_diff < 0) ? ang_diff + 360.0f : ang_diff - 360.0f;
        spec_transform.rotation.yaw = config::kSpectatorCamSmoothFactor * ang_diff + target_transform.rotation.yaw;
        spec_transform.rotation.yaw = (spec_transform.rotation.yaw >= 180.0f)
                                          ? spec_transform.rotation.yaw - 360.0f
                                          : (spec_transform.rotation.yaw <= -180.0f)
                                                ? spec_transform.rotation.yaw + 360.0f
                                                : spec_transform.rotation.yaw;
    } else
        spec_transform.rotation.yaw = config::kSpectatorCamSmoothFactor * ang_diff + target_transform.rotation.yaw;
    carla_client_->GetWorld().GetSpectator()->SetTransform(spec_transform);
}

}  // namespace agent
}  // namespace freicar
