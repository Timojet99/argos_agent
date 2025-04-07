# FreiCAR Messages
This package contains messages and services important for communication from
and to FreiCAR Agents (Mid to low level controllers responsible for driving
the cars).

It also contains definitions for `ControlReport` and `ControlCommand` messages, copied from the old `raiscar_msgs` package. The `raiscar_msgs` package is still required for launching the BaseController via `rosserial_python`.

## Note on the use of constants
The messages in this package use
[constants](https://wiki.ros.org/msg#Constants). These can be treated like
enums (which are not supported by ROS messages at the moment) and can be
accessed from within a node like in this example:

```c++
#include <freicar_msgs/AgentCommand.h>

...

if (cmd.command == freicar_msgs::AgentCommand::CMD_STOP) {
    ...
}
```

## Services
### Track.srv
The track service is a ROS service offered by the chaperone. Each new agent must
register with the chaperone upon boot and may not start driving until the
registration was successful.

#### Track Request
The track request looks like this:
```
string agent_id
bool knows_plan
bool track
```
The field `agent_id` must be set to match the *frame id* of the agent. If the
chaperone cannot find a valid transform between the `map` frame and
`agent_id`, the registration will not work.

`knows_plan` is set to true if the agent is following a plan that
will be periodically published in an AgentState message. If it is set to false,
the chaperone will try to guess the agent's trajectory by extrapolating from
transforms.

`track` must be set to true to register, and false to deregister.

#### Track Response
```
bool success
```
The track response is a single boolean `success`. The agent is not allowed to start
driving if it could not reach the chaperone or if the response is `false`

## Message Types
### AgentState.msg
This message type is intended for agents to periodically publish information
about their current state. The message type looks like this:

```
Header header
string agent_id
nav_msgs/Path current_plan
uint32 plan_index

uint8 STATE_DRIVING=0
uint8 STATE_IDLE=1
uint8 STATE_STOPPED_SELF=2
uint8 STATE_STOPPED_EXT=3
uint8 state
```

Apart from the header and the name of the agent, it contains the plan that the agent
is currently following (empty list if no path is being
followed) and the progress made, represented as an index
on that plan. The current state is encoded in the variable
`state`, which should always have the value of one of the
constants also defined in the message:

- `STATE_DRIVING (0)`: The agent is currently driving. If it is following a
  plan, the fields `current_plan` and `plan_index`
  have to be populated as described above.
- `STATE_IDLE (1)`: The agent is idle but ready to start driving.
- `STATE_STOPPED_SELF (2)`: The agent stopped itself (e.g., if
  it has gone outside the map limits) and cannot drive
further.
- `STATE_STOPPED_EXT (3)`: The agent is stopped by external
  command (e.g., because the chaperone detected an
  impending collision).

### AgentCommand.msg
This message type is used by the chaperone to send stop and resume commands to
single agents. The commands are published on a single topic, which all agents
tracked by the chaperone must subscribe to.

The message looks like this:

```
Header header
string authority
string target_agent

uint8 CMD_STOP=0
uint8 CMD_RESUME=1
uint8 command

uint8 CORRIDOR_CONFLICT=0
uint8 STATIC_OBSTACLE=0
uint8 OUT_OF_BOUNDS=1
uint8 OTHER=2
uint8 reason
```

The command is broadcast to all agents, but only the `target_agent` has to
respond. The fields `authority` and `reason` may be set to give additional
information on the command. The `command` field must be set and allows only two
values at the moment:
- `CMD_STOP (0)`: The authority wants this agent to stop immediately. Upon
  receiving this command, the agent should send an AgentState message with the
  state set to `STATE_STOPPED_EXT` to acknowledge that this command was received.
  If the chaperone stops a car because it predicted a collision, it has to know
  that the agent will stop the car in time. Otherwise the chaperone may issue
  commands to the car's hardware controller directly in an attempt to take over
  control from the agent.
- `CMD_RESUME (1)`: Lifts the last stop command. The agent may continue driving.
