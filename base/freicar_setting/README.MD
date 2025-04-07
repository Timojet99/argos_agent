# FreiCar Setting
This node is responsible applying the simulation settings & publishing a suitable clock to synchronize all nodes.

## YAML File
CARLA supports 3 different properties for simulation. they are set using the yaml file in /param.

### headless rendering
if `no-render-mode` is set to true, the simulation will run in headless mode.
### static simulated steps per second
`sim-steps-per-second` determines the desired number of simulated steps per second. to achieve acceptable physics simulation, the number should be at least 10. if set to 0, the simulation will run at full speed. this will naturally lead to variable delta time between the steps.
### synchronous mode
if `synchronous` is set to true, the simulation server will not proceed until a `Tick()` is received. Technically all CARLA clients can send it but this node is responsible for the sake of consistency.
