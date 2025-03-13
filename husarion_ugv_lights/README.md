# husarion_ugv_lights

Package used to control the Husarion UGV robot's lights.

## Launch files

This package contains:

- `lights.launch.py`: Responsible for launching the nodes required to control the robot's lights.

## Configuration Files

- [`{robot_model}_animations.yaml`](./config): Defines and describes the appearance and parameters of the animations for specific robot.
- [`{robot_model}_driver.yaml`](./config): Defines and describes specific hardware configuration for specific robot.
- [`lights_controller_parameters.yaml`](./config/lights_controller_parameters.yaml): Defines parameters for `lights_controller_node`.
- [`lights_driver_parameters.yaml`](./config/lights_driver_parameters.yaml): Defines parameters for `lights_driver_node`.

## ROS Nodes

### LightsControllerNode

This node is of type rclcpp_components is responsible for processing animations and publishing frames to `light_driver` node.

#### Publishers

- `lights/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: An animation frame to be displayed on robot Front Bumper Lights.
- `lights/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: An animation frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `lights/set_animation` [*husarion_ugv_msgs/SetLEDAnimation*]: Allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `animations_config_path` [*string*, default: **$(find husarion_ugv_lights)/husarion_ugv_lights/config/{robot_model}_animations.yaml**]: Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations.
- `controller_frequency` [*float*, default: **50.0**]: Frequency [Hz] at which the lights controller node will process animations.
- `user_led_animations_path` [*string*, default: **None**]: Path to a YAML file with a description of the user defined animations.

### LightsDriverNode

This node is of type rclcpp_components is responsible for displaying frames on the robot's lights.

#### Publishers

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Lights diagnostic messages.

#### Subscribers

- `lights/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Front Bumper Lights.
- `lights/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `lights/set_brightness` [*husarion_ugv_msgs/SetLEDBrightness*]: Allows setting global LED brightness, value ranges from **0.0** to **1.0**.

#### Service Clients

- `hardware/led_control_enable` [*std_srvs/SetBool*]: Makes SBC controlling LEDs.

#### Parameters

- `frame_timeout` [*float*, default: **0.1**]: Time in **[s]** after which an incoming frame will be considered too old.
- `global_brightness` [*float*, default: **1.0**]: LED global brightness. The range between **[0.0, 1.0]**.
- `channel_1_num_led` [*int*, default: **46**]: Number of LEDs in the first bumper.
- `channel_2_num_led` [*int*, default: **46**]: Number of LEDs in the second bumper.
