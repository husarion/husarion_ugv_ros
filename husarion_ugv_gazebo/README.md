# husarion_ugv_gazebo

The package contains a launch file and source files used to run the robot simulation in Gazebo. The simulator tries to reproduce the behavior of a real robot as much as possible, including the provision of an analogous ROS_API.

## Launch Files

- `spawn_robot.launch.py`: Responsible for spawning the robot in the simulator.
- `simulate_robot.launch.py`: Responsible for giving birth to the robot and simulating its physical behavior, such as driving, displaying data, etc.
- `simulate_multiple_robots.launch.py`: Similar to the above with logic allowing you to quickly add a swarm of robots.
- **`simulation.launch.py`**: A target file that runs the gazebo simulator that adds and simulates the robot's behavior in accordance with the given arguments.

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

Plugin based on `ign_system` is responsible for handling sensor interfaces (only IMU for now) and sending requests for joints compatible with `ros2_control`. Plugin also adds E-Stop support.

#### Publishers

- `gz_ros2_control/e_stop` [*std_msgs/Bool*]: Current E-stop state.

#### Service Servers

- `gz_ros2_control/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `gz_ros2_control/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.

> [!NOTE]
> Above topics and services should be remapped to match real robot

#### Parameters

Required parameters are defined when including the interface in the URDF (you can check out [panther_macro.urdf.xacro](../husarion_ugv_description/urdf/panther_macro.urdf.xacro)).

- `e_stop_initial_state` [*bool*, default: **true**]: Initial state of E-stop.

### LEDStrip

`LEDStrip` is a Gazebo System plugin responsible for visualizing light and displaying markers based on the data received from a `gz::msgs::Image` message.

> [!NOTE]
> The topics and services mentioned below are related to Gazebo interfaces, not ROS interfaces.

#### Subscribers

- `{topic}` [*gz::msgs::Image*]: Subscribes to an image message for visualization. The topic is specified via a parameter.

#### Service Servers

- `/marker` [*gz::msgs::Marker*]: Service to request markers for visualizing the received image.

#### Parameters

The following parameters are required when including this interface in the URDF (you can refer to the [gazebo.urdf.xacro](../husarion_ugv_description/urdf/panther/common/gazebo.urdf.xacro) file for details).

- `light_name` [*string*, default: **""**]: The name of the light entity. The visualization will be attached to this entity.
- `topic` [*string*, default: **""**]: The name of the topic from which the Image message is received.
- `namespace` [*string*, default: **""**]: Specifies the namespace to differentiate topics and models in scenarios with multiple robots.
- `frequency` [*double*, default: **10.0**]: Defines the frequency at which the animation is updated.
- `width` [*double*, default: **1.0**]: Specifies the width (y-axis) of the visualization array.
- `height` [*double*, default: **1.0**]: Specifies the height (z-axis) of the visualization array.
