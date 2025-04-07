# FreiCar CARLA Proxy
This node is responsible for providing sim functionalities in cases where the agent is completely simulated. It should *only* be run using the launch file `spawn_sim_car.launch` in `freicar_launch`.

## Parameters
* agent_name: arbitrary name for the agent

## Functionalities
The node is responsible for spawning an agent in the CARLA simulator.
* Calculating and publishing the intent corridor
* Publishing the agent status
* Subscribing to `/agent-name/control` & sending them to CARLA
* Parsing the sensor definition yaml & publishing pointclouds|depth|semseg|rgb

It also provides these features (mostly for testing):
* Subscribing to joystick controls using the `ActivateJoystickControls(str topic)` function
* Spectating the agent: the simulation camera starts following the agent (especially useful when using a joystick)

## Notes
* publishing the point cloud uses PCL under the hood (both in ROS and in this node). Therefore, the `libpcl-dev` package is required. The headers might be installed under `/usr/include/pcl-1.x/pcl` but ROS expects them to be under `/usr/include/pcl`. To get around this, you can make a simlink:
    ```bash
    sudo ln -s /usr/include/pcl-1.x/pcl /usr/include
    ```
