#include <ros/package.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <signal.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

#include "freicar_setting/settings.h"
using namespace std::chrono_literals;

// globals
carla::client::Client* carla_client = nullptr;
carla::rpc::EpisodeSettings current_settings;
ros::Publisher sim_clock;
/* connects to CARLA server */
void Connect(const std::string& address, unsigned int port) {
    try {
        carla_client = new carla::client::Client(address, port);
        carla_client->SetTimeout(10s);
        std::cout << "client version: " << carla_client->GetClientVersion() << "\t"
                  << "server version: " << carla_client->GetServerVersion() << std::endl;
        std::cout << "connected to CARLA successfully" << std::endl;
    } catch (const carla::client::TimeoutException& e) {
        ROS_ERROR("ERROR: connection to the CARLA server timed out");
        std::exit(EXIT_FAILURE);
    } catch (const std::exception& e) {
        ROS_ERROR("ERROR: %s", e.what());
        std::exit(EXIT_FAILURE);
    }
}

/* sends out clock signal & Ticks() to the simulator (if needed) */
void ClockTick(carla::client::World world, long sleep_us) {
    std::cout << "\033[1;31mWARNING: simulation is set to [Synchronous Mode]\n"
              << "killing this node will stop the simulation\033[0m" << std::endl;
    while (ros::ok()) {
        try {
            world.Tick(10s);
            ros::Time clock_time = ros::Time(world.GetSnapshot().GetTimestamp().elapsed_seconds);
            rosgraph_msgs::Clock clock_msg;
            clock_msg.clock = clock_time;
            sim_clock.publish(clock_msg);
        } catch (carla::client::TimeoutException& ex) {
            ROS_ERROR("ERROR: connection to the CARLA server timed out");
            break;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }
    std::cout << "\033[1;31mnode is shutting down. the simulation will cease to tick\n"
              << "restart simulator & dependent nodes\033[0m\n"
              << std::endl;
}

/* adjusts the current simulation settings to match the yaml file in freicar_common */
std::thread* ApplySimulationSettings() {
    std::string yaml_path = ros::package::getPath("freicar_setting") + "/param/freicar_settings.yaml";
    YAML::Node base = YAML::LoadFile(yaml_path);
    YAML::Node render_node = base["no-render-mode"];
    //	YAML::Node sync_node = base["synchronous"];
    YAML::Node step_node = base["sim-steps-per-second"];
    //	YAML::Node map_node = base["map"];
    std::string map_path;
    if (!ros::param::get("/map_path", map_path)) {
        ROS_ERROR("could not find global parameter: map_path! map initialization failed.");
        return nullptr;
    }
    bool sync_mode = false;
    if (!ros::param::get("/synchronous_mode", sync_mode)) {
        ROS_ERROR("could not find global parameter: sync_mode!");
        return nullptr;
    }
    std::string map_name = std::filesystem::path(map_path).stem();
    if (map_name == "freicar_1") {
        map_name = "freicar1";
    }
    if (map_name == "freicar_2") {
        map_name = "freicar2";
    }

    if (!render_node.IsDefined() || !step_node.IsDefined()) {
        std::cout << "ERROR: " << yaml_path << " doesn't have the correct format:"
                  << "boolean \'no-render-mode\'"
                  << "uint \'sim-steps-per-second\'" << std::endl;
        return nullptr;
    }
    if (map_name != "freicar1") {
        carla_client->LoadWorld(map_name);
        //	ros::Duration(10.0).sleep();
        std::chrono::milliseconds timespan(5000);
        std::this_thread::sleep_for(timespan);
    }

    // applying the settings to CARLA
    current_settings.no_rendering_mode = render_node.as<bool>();
    current_settings.synchronous_mode = sync_mode;
    int target_steps = step_node.as<unsigned int>();
    if (target_steps && sync_mode)
        current_settings.fixed_delta_seconds = 1.0 / target_steps;
    else
        current_settings.fixed_delta_seconds.reset();
    carla_client->GetWorld().ApplySettings(current_settings);
    std::cout << "applied simulation settings:\n"
              << "disabled rendering: " << std::to_string(current_settings.no_rendering_mode) << "\n"
              << "synchronous mode: " << std::to_string(current_settings.synchronous_mode) << "\n"
              << "simulated steps/second: " << ((target_steps == 0) ? "max" : std::to_string(target_steps))
              << std::endl;
    // setting global parameters
    ros::param::set("/sim_sps", target_steps);
    ros::param::set("/sim_sync_mode", current_settings.synchronous_mode);
    ros::param::set("/sim_norender", current_settings.no_rendering_mode);

    // ros::param::set("/use_sim_time", true);
    // for sync mode, we'd need to send tick()s
    if (current_settings.synchronous_mode) {
        long sleep_us = current_settings.fixed_delta_seconds.value() * 1000000;
        return new std::thread(ClockTick, carla_client->GetWorld(), sleep_us);
    }
    return nullptr;
}

void sig_handler(int signo) {
    (void)signo;
    if (current_settings.synchronous_mode) {
        ros::param::del("sim_sps");
        ros::param::del("sim_sync_mode");
        ros::param::del("sim_norender");
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "freicar_setting");
    ros::NodeHandle node_handle;
    sim_clock = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

    // signal handler
    signal(SIGINT, sig_handler);
    signal(SIGKILL, sig_handler);
    Connect("127.0.0.1", 2000);
    std::thread* tick_thread = ApplySimulationSettings();
    ros::spin();
    if (tick_thread && tick_thread->joinable()) {
        std::cout << "\njoining tick() thread" << std::endl;
        tick_thread->join();
    }
    return 0;
}
