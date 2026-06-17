# husarion_ugv_gazebo

The package contains a launch file and source files used to run the robot simulation in Gazebo. The simulator tries to reproduce the behavior of a real robot as much as possible, including the provision of an analogous ROS_API.

## Launch Files

- `spawn_robot.launch.py`: Responsible for spawning the robot in the simulator.
- `simulate_robot.launch.py`: Responsible for giving birth to the robot and simulating its physical behavior, such as driving, displaying data, etc.
- `simulate_multiple_robots.launch.py`: Similar to the above with logic allowing you to quickly add a swarm of robots.
- **`simulation.launch.py`**: A target file that runs the gazebo simulator that adds and simulates the robot's behavior in accordance with the given arguments.

### Worlds

To run the simulation with a different world, use:

```bash
ros2 launch husarion_ugv_gazebo simulation.launch.py gz_world:=<world>
```

Where `<world>` is either a path to an SDF file or the name of a built-in world in `husarion_gz_worlds`. The default world is `husarion_world`.

Available worlds:

- `cave`
- `empty_with_plugins`
- `husarion_office`
- `husarion_world`
- `rubicon`
- `sonoma_raceway`

## Configuration Files

- [`battery_plugin.yaml`](./config/battery_plugin.yaml): Simulated LinearBatteryPlugin configuration.
- [`gz_bridge.yaml`](./config/gz_bridge.yaml): Specify data to exchange between ROS and Gazebo simulation.
- [`teleop_with_estop.config`](./config/teleop_with_estop.config): Gazebo layout configuration file, which adds E-Stop and Teleop widgets.

## ROS Nodes

### EStop

`EStop` is a Gazebo GUI plugin responsible for easy and convenient changing of the robot's E-stop state.

#### Service Clients

- `hardware/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.

### EStopSystem

`EStopSystem` is a standalone Gazebo System plugin that reproduces the robot E-stop interface in simulation. While the E-stop is active, it overrides the joint velocity commands written by `gz_ros2_control` with zeros so the robot stops. It must be declared in the URDF after the `gz_ros2_control` plugin (systems execute in declaration order).

#### Publishers

- `hardware/e_stop` [*std_msgs/Bool*]: Current E-stop state (latched).

#### Service Servers

- `hardware/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.

#### Parameters

Parameters are defined when including the plugin in the URDF (see the `estop` macro in [gazebo.urdf.xacro](../husarion_ugv_description/urdf/common/gazebo.urdf.xacro)).

- `namespace` [*string*, default: **""**]: Namespace of the ROS node, topic and services.
- `initial_state` [*bool*, default: **true**]: Initial state of E-stop.

### LEDStrip

`LEDStrip` is a Gazebo System plugin responsible for visualizing light and displaying markers based on the data received from a `gz::msgs::Image` message.

> [!NOTE]
> The topics and services mentioned below are related to Gazebo interfaces, not ROS interfaces.

#### Subscribers

- `{topic}` [*gz::msgs::Image*]: Subscribes to an image message for visualization. The topic is specified via a parameter.

#### Service Servers

- `/marker_array` [*gz::msgs::Marker_V*]: Service to request the markers visualizing the received image (all of a frame's changed markers are sent in a single batched request).

#### Parameters

The following parameters are required when including this interface in the URDF (you can refer to the [gazebo.urdf.xacro](../husarion_ugv_description/urdf/common/gazebo.urdf.xacro) file for details).

- `light_name` [*string*, default: **""**]: The name of the light entity. The visualization will be attached to this entity.
- `topic` [*string*, default: **""**]: The name of the topic from which the Image message is received.
- `namespace` [*string*, default: **""**]: Specifies the namespace to differentiate topics and models in scenarios with multiple robots.
- `frequency` [*double*, default: **10.0**]: Defines the frequency at which the animation is updated.
- `width` [*double*, default: **1.0**]: Specifies the width (y-axis) of the visualization array (used when `points` is not set).
- `height` [*double*, default: **1.0**]: Specifies the height (z-axis) of the visualization array.
- `led_range` [*string*, default: **whole frame**]: `<first>-<last>` subrange of the channel frame displayed by this instance. A reversed range (e.g. `23-12`) flips the LED order.
- `points` [*string*, default: **""**]: Polyline (`x y z; x y z; ...`, in the light frame) the LEDs are distributed along. When empty, the strip is a straight line along the Y axis of the given width.
- `render_markers_per_led` [*int*, default: **1**]: Number of rendered marker cells per LED used to interpolate the diffuser gradient. `1` keeps sharp per-LED markers; higher values smooth the transition between neighboring LEDs at the cost of proportionally more markers.
- `strip_base_color` [*string*, default: **"1 1 1"**]: RGB color the strip is composited over, so animation transparency reveals it (a white diffuser) instead of blending into the black scene.
