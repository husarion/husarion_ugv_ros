# husarion_ugv_ros

ROS 2 packages for Husarion UGV (Unmanned Ground Vehicle). The repository is a collection of necessary packages enabling the launch of the Lynx and Panther robots.

[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_with_lights.png">
  <img alt="Panther preview" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/panther_ros/day_no_lights.png">
</picture>

## Quick start

### Create workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone -b ros2 https://github.com/husarion/husarion_ugv_ros.git src/husarion_ugv_ros
```

### Configure environment

The repository is used to run the code both on the real robot and in the simulation. Specify `HUSARION_ROS_BUILD_TYPE` the variable according to your needs.

Real robot:

``` bash
export HUSARION_ROS_BUILD_TYPE=hardware
```

Simulation:

```bash
export HUSARION_ROS_BUILD_TYPE=simulation
```

### Build

``` bash
vcs import src < src/husarion_ugv_ros/husarion_ugv/${HUSARION_ROS_BUILD_TYPE}_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

>[!NOTE]
> To build code on a real robot you need to run above commands on the robot Built-in Computer.

### Running

Real robot:

```bash
ros2 launch husarion_ugv_bringup bringup.launch.py
```

Simulation:

```bash
ros2 launch husarion_ugv_gazebo simulation.launch.py
```

> [!IMPORTANT]
> You can change spawning robot in simulation, by adding `robot_model:={robot_model}` argument.

### Launch Arguments

Launch arguments are largely common to both simulation and physical robot. However, there is a group of arguments that apply only to hardware or only to the simulator. Below is a legend to the tables with all launch arguments.

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulation      |

| 🤖   | 🖥️   | Argument                     | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                            |
| --- | --- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| ❌   | ✅   | `add_world_transform`        | Adds a world frame that connects the tf trees of individual robots (useful when running multiple robots). <br/> ***bool:*** `False`                                                                                                                                                                                |
| ✅   | ✅   | `animations_config_path`     | Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations. <br/> ***string:*** [`{robot_model}_animations.yaml`](./husarion_ugv_lights/config)                                                                           |
| ✅   | ✅   | `battery_config_path`        | Path to the Gazebo `LinearBatteryPlugin` configuration file in simulation. On the robot it is only a fallback for `battery_driver_config_path`, kept for launch commands that used it. <br/> ***string:*** `None`                                                                                                                                                                |
| ✅   | ❌   | `battery_driver_config_path` | Path to the battery driver configuration file. A file given here replaces the default one, it is not merged with it. Falls back to `battery_config_path` when only that is given on the command line. <br/> ***string:*** [`battery.yaml`](./husarion_ugv_battery/config/battery.yaml) |
| ✅   | ✅   | `components_config_path`     | Additional components configuration file. Components described in this file are dynamically included in robot's URDF. Available options are described in [the manual](https://husarion.com/manuals/panther/panther-options). <br/> ***string:*** [`components.yaml`](./husarion_ugv_description/config/components.yaml) |
| ✅   | ✅   | `controller_config_path`     | Path to controller configuration file. A path to custom configuration can be specified here. <br/> ***string:*** [`{wheel_type}_controller.yaml`](./husarion_ugv_controller/config/)                                                                                                                               |
| ✅   | ❌   | `controller_params_overlay_path` | Parameter file that `ros2_control_node` loads after `controller_config_path`, so its values override the ones there, e.g. `controller_manager` thread settings or a broadcaster `update_rate`. Entries are matched by node name like in the controller configuration file. An empty value loads none, a path that does not exist stops the launch. <br/> ***string:*** `""` |
| ✅   | ✅   | `disable_manager`            | Enable or disable manager_bt_node. <br/> ***bool:*** `False`                                                                                                                                                                                                                                                       |
| ✅   | ❌   | `ekf_print_diagnostics`      | Sets `print_diagnostics` of the EKF, which robot_localization needs for its `odometry/filtered` frequency diagnostic. <br/> ***bool:*** `False`                                                                                                                                                                    |
| ✅   | ✅   | `fuse_gps`                   | Include GPS for data fusion. <br/> ***bool:*** `False`                                                                                                                                                                                                                                                             |
| ✅   | ✅   | `groot_enabled`              | Start the Groot2 publishers of the lights, safety and shutdown behavior trees. Each one keeps a ZMQ server and a heartbeat thread running, so it can be turned off when Groot2 is not used. <br/> ***bool:*** `True` |
| ❌   | ✅   | `gz_bridge_config_path`      | Path to the parameter_bridge configuration file. <br/> ***string:*** [`gz_bridge.yaml`](./husarion_ugv_gazebo/config/gz_bridge.yaml)                                                                                                                                                                               |
| ❌   | ✅   | `gz_gui`                     | Run simulation with specific GUI layout. <br/> ***string:*** [`teleop.config`](https://github.com/husarion/husarion_gz_worlds/blob/main/config/teleop.config)                                                                                                                                                      |
| ❌   | ✅   | `gz_headless_mode`           | Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the number of calculations. <br/> ***bool:*** `False`                                                                                                                                                                            |
| ❌   | ✅   | `gz_log_level`               | Adjust the level of console output. <br/> ***int:*** `1` (choices: `0`, `1`, `2`, `3`, `4`)                                                                                                                                                                                                                        |
| ❌   | ✅   | `gz_world`                   | Absolute path to SDF world file. <br/> ***string:*** [`husarion_world.sdf`](https://github.com/husarion/husarion_gz_worlds/blob/main/worlds/husarion_world.sdf)                                                                                                                                                    |
| ✅   | ✅   | `launch_gamepad`            | Enable robot control via gamepad. <br/> ***bool:*** `True` for physical robot; <br/> ***bool:*** `False` for simulation                                                                                                     |
| ✅   | ✅   | `launch_nmea_gps`            | Whether to launch the NMEA NavSat driver node. Advisable when the robot is equipped with the [ANT02](https://husarion.com/manuals/panther/panther-options/#ant02---wi-fi--lte--gps). <br/> ***bool:*** `False`                                                                                                     |
| ✅   | ❌   | `led_control_retry_forever`  | Keep asking the hardware for LED control until it is granted, instead of exiting after three attempts. <br/> ***bool:*** `False`                                                                                                                                                                                  |
| ✅   | ❌   | `led_spi_max_transfer_bytes` | Largest single SPI transfer of a LED frame in bytes. **0** sends each frame as one transfer. <br/> ***int:*** `0` |
| ✅   | ❌   | `led_spi_speed_hz`           | SPI clock of the LED strips in Hz. **0** keeps the driver default of 800000, the certified clock. Changing it affects EMC. <br/> ***int:*** `0`                                                                                                                                                              |
| ✅   | ✅   | `lights_bt_project_path`     | Path to BehaviorTree project file, responsible for lights management. <br/> ***string:*** [`LightsBT.btproj`](./husarion_ugv_manager/behavior_trees/LightsBT.btproj)                                                                                                                                 |
| ✅   | ❌   | `lights_controller_frequency` | Frequency in Hz at which the lights controller renders animation frames. <br/> ***float:*** `50.0` |
| ✅   | ❌   | `lights_frame_keepalive_period` | Time in seconds after which the lights controller publishes an unchanged frame again, at most 5. **0** publishes every frame. <br/> ***float:*** `0.0` |
| ✅   | ✅   | `localization_config_path`   | Specify the path to the localization configuration file. <br/> ***string:*** [`relative_localization.yaml`](./husarion_ugv_localization/config/relative_localization.yaml)                                                                                                                                         |
| ✅   | ✅   | `localization_mode`          | Specifies the localization mode:  <br/>- 'relative' `odometry/filtered` data is relative to the initial position and orientation. <br/>- 'enu' `odometry/filtered` data is relative to initial position and ENU (East North Up) orientation. <br/> ***string:*** `relative` (choices: `relative`, `enu`)           |
| ✅   | ✅   | `log_level`                  | Sets verbosity of launched nodes. <br/> ***string:*** `INFO` (choices: `DEBUG`, `INFO`, `WARN`, `WARNING` (alias of `WARN`), `ERROR`, `FATAL`)                                                                                                                               |
| ✅   | ✅   | `namespace`                  | Add namespace to all launched nodes. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                                                                                    |
| ❌   | ✅   | `robot_model`                | Specify robot model type. <br/> ***string:*** `env(ROBOT_MODEL_NAME)` (choices: `lynx`, `panther`)                                                                                                                                                                                                                      |
| ✅   | ✅   | `safety_bt_project_path`     | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/> ***string:*** [`SafetyBT.btproj`](./husarion_ugv_manager/behavior_trees/SafetyBT.btproj)                                                                                                                    |
| ✅   | ✅   | `shutdown_hosts_config_path` | Path to file with list of hosts to request shutdown. <br/> ***string:*** [`shutdown_hosts.yaml`](./husarion_ugv_manager/config/shutdown_hosts.yaml)                                                                                                                                                                |
| ✅   | ✅   | `use_ekf`                    | Enable or disable EKF. <br/> ***bool:*** `True`                                                                                                                                                                                                                                                                    |
| ❌   | ✅   | `use_joint_state_publisher`           | Flag enabling joint_state_publisher to publish information about the wheel position. Should be false when there is a controller that sends this information. <br/> ***bool:*** `False`                                                                                                                             |
| ❌   | ✅   | `use_joint_state_publisher_gui`           | Flag enabling joint_state_publisher_gui to publish information about the wheel position. Should be false when there is a controller that sends this information. <br/> ***bool:*** `False`                                                                                                                             |
| ✅   | ❌   | `use_madgwick_filter`        | Determine the IMU orientation with the Madgwick filter. <br/> ***bool:*** `False` |
| ✅   | ❌   | `use_mag`                    | Use the IMU magnetometer in the Madgwick filter. Has an effect only with `use_madgwick_filter:=True` (off by default). The stock IMU sits inside the chassis next to the motors, where the magnetometer reading is disturbed. <br/> ***bool:*** `True` |
| ❌   | ✅   | `use_rviz`                   | Run RViz simultaneously. <br/> ***bool:*** `True`                                                                                                                                                                                                                                                                                   |
| ✅   | ✅   | `use_sim`                    | Whether simulation is used. <br/> ***bool:*** `False`                                                                                                                                                                                                                                                              |
| ✅   | ✅   | `user_led_animations_path`   | Path to a YAML file with a description of the user-defined animations. <br/> ***string:*** `''`                                                                                                                                                                                                                    |
| ✅   | ✅   | `wheel_config_path`          | Path to wheel configuration file. <br/> ***string:*** [`{wheel_type}.yaml`](./husarion_ugv_description/config)                                                                                                                                                                                                          |
| ✅   | ✅   | `wheel_type`                 | Specify the wheel type. If the selected wheel type is not 'custom', the wheel_config_path and controller_config_path arguments will be automatically adjusted and can be omitted. <br/> ***string:*** `WH01` (for Panther), `WH05` (for Lynx) (choices: `WH01`, `WH02`, `WH04`, `WH05`, `custom`)                  |
| ❌   | ✅   | `x`                          | Initial robot position in the global 'x' axis. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                            |
| ❌   | ✅   | `y`                          | Initial robot position in the global 'y' axis. <br/> ***float:*** `-2.0`                                                                                                                                                                                                                                           |
| ❌   | ✅   | `z`                          | Initial robot position in the global 'z' axis. <br/> ***float:*** `0.2`                                                                                                                                                                                                                                            |
| ❌   | ✅   | `roll`                       | Initial robot 'roll' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                         |
| ❌   | ✅   | `pitch`                      | Initial robot 'pitch' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                        |
| ❌   | ✅   | `yaw`                        | Initial robot 'yaw' orientation. <br/> ***float:*** `0.0`                                                                                                                                                                                                                                                          |

> [!TIP]
>
> To read the arguments for individual packages, add the `-s` flag to the `ros2 launch` command (e.g. `ros2 launch husarion_ugv_bringup bringup.launch.py ​​-s`)

## Developer Info

### Setup pre-commit

This project uses pre-commit to maintain high quality of the source code. Install precommit after downloading the repository to apply the changes.

```bash
pre-commit install
```

### Unit testing

#### Running on laptop

```bash
colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DTEST_INTEGRATION=OFF
colcon test --packages-up-to husarion_ugv
```

#### Running on the Built-In Computer

```bash
colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DTEST_INTEGRATION=ON
colcon test --packages-up-to husarion_ugv
```
