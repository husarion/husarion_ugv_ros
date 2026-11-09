# husarion_ugv_lights

Package used to control the Husarion UGV robot's lights.

## Launch files

This package contains:

- `lights.launch.py`: Responsible for launching the nodes required to control the robot's lights.

### Launch arguments

- `led_control_retry_forever` [*bool*, default: **False**]: Passed to the `led_control_retry_forever` parameter of `LightsDriverNode`. `bringup.launch.py` forwards it.
- `led_spi_speed_hz` [*int*, default: **0**]: Passed to the `spi_speed_hz` parameter of `LightsDriverNode`: **0** or a clock between **100000** and **20000000** Hz. Also a `husarion_ugv_bringup` argument.
- `led_spi_max_transfer_bytes` [*int*, default: **0**]: Passed to the `spi_max_transfer_bytes` parameter of `LightsDriverNode`. Opt-in and meant to save CPU; its effect has not been measured, so leave it at **0** unless you measure it.
- `lights_controller_frequency` [*float*, default: **50.0**]: Passed to the `controller_frequency` parameter of `LightsControllerNode`. Animations are timed in seconds, so a lower rate drops frames and keeps their timing, but a pulse shorter than two frames can disappear.
- `lights_frame_keepalive_period` [*float*, default: **0.0**]: Passed to the `frame_keepalive_period` parameter of `LightsControllerNode`.

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

With `frame_keepalive_period` above 0 a frame is published when it changes and at least once per period. The topics are not latched, so a subscriber that joins late, such as `ros2 topic echo` or a simulation bridge, sees nothing until the next change or keep-alive. A frame the driver dropped, for example one that arrived after `frame_timeout`, is not repeated until the period expires, so the strip can stay one frame behind for up to that long. The period is capped at 5 s for that reason.

#### Service Servers

- `lights/set_animation` [*husarion_ugv_msgs/SetLEDAnimation*]: Allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `animations_config_path` [*string*, default: **$(find husarion_ugv_lights)/husarion_ugv_lights/config/{robot_model}_animations.yaml**]: Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations.
- `controller_frequency` [*float*, default: **50.0**]: Frequency [Hz] at which the lights controller node will process animations.
- `frame_keepalive_period` [*float*, default: **0.0**]: Time [s] after which an unchanged frame is published again, between **0.0** and **5.0**. Above 0 frames are otherwise published only when they change. **0** publishes every frame.
- `user_led_animations_path` [*string*, default: **None**]: Path to a YAML file with a description of the user defined animations.

### LightsDriverNode

This node is of type rclcpp_components is responsible for displaying frames on the robot's lights.

#### Publishers

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Lights diagnostic messages. The `Lights driver status` task shows the LED control state, the `lights/enable` output state and the brightness. It reports the frames dropped since its last report and the last reason; frames rejected while LED control is not granted yet are not counted. By default every dropped frame also broadcasts a WARN of its own; with `report_frame_drops_periodically` the task warns instead when more than 2 were dropped in one report, that is above 2 per second with the default `diagnostic_updater.period` of 1 s.
- `lights/output_state` [*husarion_ugv_msgs/LEDOutputState*, latched]: State of the physical LED output: the `lights/enable` switch, whether LED control is granted and the global brightness. Published on every change.

#### Subscribers

- `lights/channel_1_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Front Bumper Lights.
- `lights/channel_2_frame` [*sensor_msgs/Image*, encoding: **RGBA8**, height: **1**, width: **num_led**]: Frame to be displayed on robot Rear Bumper Lights.

#### Service Servers

- `lights/enable` [*std_srvs/SetBool*]: Switches the physical LED output. While off, frames are still received and checked, nothing is written to the strips, and the strips are blanked once so they do not keep the last frame. Switching it on writes the latest frame again. The LED control grant also writes the latest frame after clearing the strips, so a frame that arrived before the grant is shown.
- `lights/set_brightness` [*husarion_ugv_msgs/SetLEDBrightness*]: Allows setting global LED brightness, value ranges from **0.0** to **1.0**. With LED control granted and the output on, the latest frame is written again with the new brightness. If that write fails, the call still returns `success = true`, since the brightness is applied to the next frame, and `message` says that the write failed.

#### Service Clients

- `hardware/led_control_enable` [*std_srvs/SetBool*]: Makes SBC controlling LEDs.

#### Parameters

- `frame_timeout` [*float*, default: **0.1**]: Time in **[s]** after which an incoming frame will be considered too old.
- `global_brightness` [*float*, default: **1.0**]: LED global brightness. The range between **[0.0, 1.0]**.
- `led_output_enabled` [*bool*, default: **true**]: Startup state of the physical LED output, see the `lights/enable` service.
- `led_control_retry_forever` [*bool*, default: **false**]: Keep asking for LED control at startup until the hardware grants it. With **false** the driver gives up after three attempts and the lights container exits, which stops the launch. It covers startup only: control lost later, for example when the hardware interface restarts, is not asked for again.
- `report_frame_drops_periodically` [*bool*, default: **false**]: With **false** every rejected frame broadcasts a WARN diagnostic, as before. With **true** rejected frames are only counted, and the `Lights driver status` task warns when more than 2 were dropped since its last report.
- `channel_1_num_led` [*int*, default: **46**]: Number of LEDs in the first bumper.
- `channel_2_num_led` [*int*, default: **46**]: Number of LEDs in the second bumper.
- `spi_speed_hz` [*int*, default: **0**]: SPI clock of both LED channels in **[Hz]**. **0** keeps the default of 800000, the clock the robots are certified with. Any other value must be between **100000** and **20000000**. Otherwise the driver fails to load with an error naming the parameter: the container and the controller keep running, and the strips stay dark, as with a missing SPI device. Changing it affects EMC. `lights.launch.py` sets it from the `led_spi_speed_hz` launch argument, so it is not read from `{robot_model}_driver.yaml`. The Raspberry Pi runs the next lower clock its even divider of the core clock can reach, and at start the driver logs the requested clock and the clock that divider gives at the default core clock.
- `spi_max_transfer_bytes` [*int*, default: **0**]: Largest single SPI transfer in bytes. A longer frame is sent as several transfers of one message, so on a Raspberry Pi a value below 96 keeps the frame off DMA. **0** sends each frame as one transfer. `lights.launch.py` sets it from the `led_spi_max_transfer_bytes` launch argument.
