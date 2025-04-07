# FreiCAR Chaperone
This node is intended for coordinating multiple real-world agents to detect and
prevent collisions.

The chaperone node runs on the base computer and can track multiple running cars
based on agent state messages or TF. It tries to detect and prevent collisions
by estimating future car movements and sending stop commands when there is an
intersection in future paths. It is also capable of force-stopping the cars if
the agent does not react to the stop command.

The chaperone can also be used to prevent cars from going off-track by reading
a pre-recorded bagfile `data/fence.bag`, which contains a recording of the
track outline.

## Usage
### Recording the fence.bag
If you need to record a new map outline for the off-track detection, turn on
the `mapping_tool` Vive tracker, which is the Vive tracker on a stick with a
wheel. Then run:

```
roscd freicar_chaperone
rm data/fence.bag
./record-fence.sh
```

Wait 10 seconds, then move the mapping tool in a smooth motion over the desired
map limits. All TFs will be recorded and the `/mapping_tool` poses will be used
to compute the fence. There is no need to fully close the loop, since a polygon
will be extracted later on. If you want to allow the cars to go a little off
road, leave a gap.

### Launching
Run the node with:

```
roslaunch freicar_chaperone freicar_chaperone.launch
```

See the comments in the launch file for details on the parameters. The node
will offer a service where agents need to register (see `freicar_msgs/srv`) and
publish visualization messages for Rviz.
