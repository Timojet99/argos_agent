# FreiCAR Agent

This package contains an example agent showing the minimum required communication
with the chaperone, a node that tries to predict and prevent collisions between
agents.

## Starting the agent
After compiling, the agent may be started using the provided launch files:
1. `roslaunch freicar_agent freicar_agent_hw.launch` to start services on the actual car, e.g., the low-level
controller to control the actuators
2. `roslaunch freicar_agent freicar_agent.launch` to start your code that will send throttle and steering
commands to the low-level controller

**NOTE:**
You have to set the agent name in both launch files, e.g., for *freicar_1* change this line:
`<arg name="agent_name" default="freicar_1" />`


## Parameters
Each required parameter with default value in parentheses:

- `agent_name`: The agent name. Must always match the controlled car's frame id!
- `global_frame ("map")`: Not strictly required by this example agent, but will be useful for obtaining TFs
    relative to the map frame when doing actual controlling.
- `agent_state_topic ("/$(arg car_name)/agent_state")`: The topic where the agent publishes state information
for the chaperone and other nodes.
- `cmd_topic ("/freicar_agent_commands")`: The global topic where the agents receive commands from the chaperone.
Must be subscribed to.
- `track_service ("/freicar_chaperone_track_srv")`: The service ID where the chaperone listens for track requests.
- `requires_chaperone (true)`: The agent is only allowed to drive after registration to the chaperone. You can disable
this for milestone 4.
- `safety_radius (0.1)`: The agent should stop itself if it leaves the drivable area by more than this margin.
- `safety_localization_time (0.5)`: The agent should stop itself if the latest localization data is older than this
many seconds.


## Code explained

### Communication with the chaperone
In the challenge, multiple cars might be on the map at the same time. To avoid collisions caused by malfunctioning
code on the cars, a chaperone server supervises the agents' paths and stops them if required. Thus, agents are only
allowed to drive after they registered to the chaperone. The overall flow looks like this

1. `RegisterToChaperone()` sets `is_registered` to true. The car is now being tracked by the chaperone and can start
driving.
2. Run one step of your control code.
3. `PublishState()` to the chaperone to update it about the current plan and state (see message package).
4. Repeat steps 2 and 3 as long as this node is running.
5. `DeregisterFromChaperone()` sets `is_registered` to false. The car is not being tracked anymore and must not drive.

Additionally,
- The `ProcessCommand()` callback reacts to instructions from the chaperone, e.g., stopping the car.
- For milestone 4, the chaperone should be disabled by setting the `requires_chaperone` parameter to false in the
launch file.


## General notes

You are free to change the code in this skeleton the way you like as long as your agent registers to the chaperone
and you do not implement any shortcuts to ignore commands from the chaperone.

The communication with the chaperone should only be changed in the designated lines marked by `TODO`. Most other
member functions can be considered as ideas or proposals how you could implement them. You are definitely allowed to
add more functions and variables. If in doubt, ask the TAs.
