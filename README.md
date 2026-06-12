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
ros2 launch husarion_ugv_bringup bringup.launch.yaml
```

Simulation:

```bash
ros2 launch husarion_ugv_gazebo simulation.launch.yaml
```

> [!IMPORTANT]
> You can change which robot is spawned in simulation by passing the `robot_model:={robot_model}` argument.

> [!NOTE]
> The previous Python entry-points (`bringup.launch.py`, `simulation.launch.py`) still
> work as backwards-compatibility shims that print a deprecation warning. New code and
> tutorials should use the `.launch.yaml` files directly.

### Launch Arguments

Launch arguments are largely common to both simulation and physical robot. However, there is a group of arguments that apply only to hardware or only to the simulator. Below is a legend to the tables with all launch arguments.

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulation      |

**Configuration overrides.** All path-style arguments default to files in the relevant
package's `config/` directory. To override the entire config tree without forking the
packages, set `common_dir_path` to a directory mirroring the package layout — paths are
then resolved as `<common_dir_path>/<package_name>/config/<file>.yaml` (e.g.
`<common_dir_path>/husarion_ugv_battery/config/battery.yaml`).

**Namespacing.** `bringup.launch.yaml` and `simulation.launch.yaml` wrap the robot's
nodes in a `push_ros_namespace` group and use `set_remap` entries for `/tf`, `/tf_static`, and `/diagnostics`. When
`namespace` is non-empty, the optional `tf_namespace_bridge` node (enabled by default)
republishes `/<namespace>/tf` to the global `/tf` with a frame prefix — required for
RViz/visualizers running outside the namespace. Disable with `tf_namespace_bridge:=False`.

**Frame names vs topic namespacing.** By default, frame names stay literal (e.g.
`base_link`, `camera_link`) — the namespace lives only on the *topic* (e.g.
`/<namespace>/tf`). This is observable in simulation by echoing any sensor topic:

```bash
ros2 topic echo /<namespace>/camera/image_raw --field header.frame_id
# default (components_use_tf_prefix:=False):
camera_link
# with components_use_tf_prefix:=True:
<namespace>/camera_link
```

The same applies to `/<namespace>/scan`, `/<namespace>/imu/data`, etc. Toggle via the
top-level `components_use_tf_prefix` arg (forwarded automatically through the include
chain to xacro). On hardware the default workflow runs without a namespace, so the
prefix is empty either way — practically a sim-only knob. Keep `False` whenever the
`push_ros_namespace` + `/tf` remap model is in use: mixing literal robot-body frames
with prefixed component frames disconnects the TF tree.

| 🤖   | 🖥️   | Argument                     | Description <br/> ***Type:*** `Default`                                                                                                                                                                                                                                                                            |
| --- | --- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| ❌   | ✅   | `add_world_transform`        | Adds a world frame that connects the tf trees of individual robots (useful when running multiple robots). <br/> ***bool:*** `False`                                                                                                                                                                                |
| ✅   | ✅   | `animations_config_path`     | Path to a YAML file with a description of led configuration. This file includes definition of robot panels, virtual segments and default animations. <br/> ***string:*** [`{robot_model}_animations.yaml`](./husarion_ugv_lights/config)                                                                           |
| ✅   | ✅   | `battery_config_path`        | Path to the battery configuration file. **Hardware** (`bringup.launch.yaml`): the battery driver config — [`battery.yaml`](./husarion_ugv_battery/config/battery.yaml). **Simulation** (`simulation.launch.yaml`): the Gazebo `LinearBatteryPlugin` config — [`battery_plugin.yaml`](./husarion_ugv_gazebo/config/battery_plugin.yaml). <br/> ***string:*** depends on context |
| ✅   | ✅   | `common_dir_path`            | When set, configs are resolved from `<common_dir_path>/<package_name>/...` instead of the package share dir, allowing centralized overrides across all packages without forking each one. <br/> ***string:*** `''`                                                                                                  |
| ✅   | ✅   | `components_config_path`     | Additional components configuration file. Components described in this file are dynamically included in robot's URDF. Available options are described in [the manual](https://husarion.com/manuals/panther/panther-options). <br/> ***string:*** [`components.yaml`](./husarion_ugv_description/config/components.yaml) |
| ❌   | ✅   | `components_use_tf_prefix`   | Prefix every component's `frame_id` with the robot namespace inside the URDF. Observable in simulation as `header.frame_id` on any sensor topic: `False` → `camera_link`; `True` → `<namespace>/camera_link`. On hardware the default workflow runs without a namespace, so the prefix is empty either way — kept for parity with simulation. Setting `True` while keeping the default `push_ros_namespace` + `/tf` remap model mixes literal robot-body frames with prefixed component frames and disconnects the TF tree. <br/> ***bool:*** `False`                                                                                                          |
| ✅   | ✅   | `controller_config_path`     | Path to controller configuration file. A path to custom configuration can be specified here. <br/> ***string:*** [`{wheel_type}_controller.yaml`](./husarion_ugv_controller/config/)                                                                                                                               |
| ✅   | ✅   | `disable_manager`            | Enable or disable manager_bt_node. <br/> ***bool:*** `False`                                                                                                                                                                                                                                                       |
| ✅   | ❌   | `exit_on_wrong_hw`           | Exit the launch if the hardware configuration is detected as incorrect. When `False`, the launch hangs on `sleep infinity` after printing an error so the launch session stays alive for debugging. <br/> ***bool:*** `False` |
| ✅   | ✅   | `fuse_gps`                   | Include GPS for data fusion. <br/> ***bool:*** `False`                                                                                                                                                                                                                                                             |
| ❌   | ✅   | `gz_bridge_config_path`      | Path to the parameter_bridge configuration file. <br/> ***string:*** [`gz_bridge.yaml`](./husarion_ugv_gazebo/config/gz_bridge.yaml)                                                                                                                                                                               |
| ❌   | ✅   | `gz_gui`                     | Run simulation with specific GUI layout. <br/> ***string:*** [`teleop.config`](https://github.com/husarion/husarion_gz_worlds/blob/main/config/teleop.config)                                                                                                                                                      |
| ❌   | ✅   | `gz_headless_mode`           | Run the simulation in headless mode. Useful when a GUI is not needed or to reduce the number of calculations. <br/> ***bool:*** `False`                                                                                                                                                                            |
| ❌   | ✅   | `gz_log_level`               | Adjust the level of console output. <br/> ***int:*** `1` (choices: `0`, `1`, `2`, `3`, `4`)                                                                                                                                                                                                                        |
| ❌   | ✅   | `gz_world`                   | Absolute path to SDF world file. <br/> ***string:*** [`husarion_world.sdf`](https://github.com/husarion/husarion_gz_worlds/blob/main/worlds/husarion_world.sdf)                                                                                                                                                    |
| ✅   | ✅   | `joy2twist_params_file`      | Path to the joy2twist params file (bubbled from `teleop.launch.yaml`). <br/> ***string:*** [`joy2twist_{robot_model}.yaml`](./husarion_ugv_teleop/config)                                                                                                                                                       |
| ✅   | ✅   | `launch_gamepad`            | Enable robot control via gamepad. <br/> ***bool:*** `True` for physical robot; <br/> ***bool:*** `False` for simulation                                                                                                     |
| ✅   | ✅   | `launch_nmea_gps`            | Whether to launch the NMEA NavSat driver node. Advisable when the robot is equipped with the [ANT02](https://husarion.com/manuals/panther/panther-options/#ant02---wi-fi--lte--gps). <br/> ***bool:*** `False`                                                                                                     |
| ✅   | ✅   | `lights_bt_project_path`     | Path to BehaviorTree project file, responsible for lights management. <br/> ***string:*** [`LightsBT.btproj`](./husarion_ugv_manager/behavior_trees/LightsBT.btproj)                                                                                                                                 |
| ✅   | ✅   | `localization_config_path`   | Specify the path to the localization configuration file. <br/> ***string:*** [`relative_localization.yaml`](./husarion_ugv_localization/config/relative_localization.yaml)                                                                                                                                         |
| ✅   | ✅   | `localization_mode`          | Specifies the localization mode:  <br/>- 'relative' `odometry/filtered` data is relative to the initial position and orientation. <br/>- 'enu' `odometry/filtered` data is relative to initial position and ENU (East North Up) orientation. <br/> ***string:*** `relative` (choices: `relative`, `enu`)           |
| ✅   | ✅   | `log_level`                  | Sets verbosity of launched nodes. <br/> ***string:*** `INFO`
| ✅   | ✅   | `namespace`                  | Add namespace to all launched nodes. Pushed via `push_ros_namespace` inside the per-robot group. <br/> ***string:*** `env(ROBOT_NAMESPACE)`                                                                                                                                                                       |
| ✅   | ✅   | `nmea_params_path`           | Path to the NMEA NavSat driver params file (bubbled from `nmea_navsat.launch.yaml`). <br/> ***string:*** [`nmea_navsat.yaml`](./husarion_ugv_localization/config/nmea_navsat.yaml)                                                                                                                                |
| ❌   | ✅   | `robot_model`                | Specify robot model type. <br/> ***string:*** `env(ROBOT_MODEL_NAME)` (choices: `lynx`, `panther`)                                                                                                                                                                                                                      |
| ❌   | ✅   | `rviz_config`                | Path to the RViz config file (bubbled from `rviz.launch.yaml` inside the per-robot group). <br/> ***string:*** [`husarion_ugv.rviz`](./husarion_ugv_description/rviz/husarion_ugv.rviz)                                                                                                                            |
| ✅   | ✅   | `safety_bt_project_path`     | Path to BehaviorTree project file, responsible for safety and shutdown management. <br/> ***string:*** [`SafetyBT.btproj`](./husarion_ugv_manager/behavior_trees/SafetyBT.btproj)                                                                                                                    |
| ✅   | ✅   | `shutdown_hosts_config_path` | Path to file with list of hosts to request shutdown. <br/> ***string:*** [`shutdown_hosts.yaml`](./husarion_ugv_manager/config/shutdown_hosts.yaml)                                                                                                                                                                |
| ✅   | ❌   | `system_monitor_config_path` | Path to the system monitor configuration file (bubbled from `system_monitor.launch.yaml`; only included in hardware bringup). <br/> ***string:*** [`system_monitor.yaml`](./husarion_ugv_diagnostics/config/system_monitor.yaml)                                                                                |
| ✅   | ✅   | `tf_namespace_bridge`        | Bridge robot's namespaced TF (`/<namespace>/tf`, `/<namespace>/tf_static`) to the global `/tf` and `/tf_static` with a frame prefix. Required for visualization tools subscribing to `/tf` outside the namespace. Active only when `namespace` is non-empty. <br/> ***bool:*** `True`                            |
| ✅   | ✅   | `tf_namespace_bridge_config` | Path to the `tf_namespace_bridge` configuration file (frame filters, etc.). <br/> ***string:*** [`tf_namespace_bridge.yaml`](./husarion_ugv_bringup/config/tf_namespace_bridge.yaml) for HW; [`tf_namespace_bridge.yaml`](./husarion_ugv_gazebo/config/tf_namespace_bridge.yaml) for Sim                          |
| ✅   | ✅   | `use_ekf`                    | Enable or disable EKF. <br/> ***bool:*** `True`                                                                                                                                                                                                                                                                    |
| ❌   | ❌   | `use_joint_state_publisher`           | Flag enabling joint_state_publisher to publish wheel positions (no real controller). **Available only via `visualize_fake_robot.launch.yaml`** — not exposed by the main bringup or simulation entry points. <br/> ***bool:*** `False`                                                                              |
| ❌   | ❌   | `use_joint_state_publisher_gui`           | Flag enabling joint_state_publisher_gui to publish wheel positions (no real controller). **Available only via `visualize_fake_robot.launch.yaml`**. <br/> ***bool:*** `False`                                                                                                                                       |
| ✅   | ✅   | `use_madgwick_filter`        | Determine orientation from IMU using the Madgwick filter (bubbled from `controller.launch.yaml` / `load_urdf.launch.yaml`). When `True`, IMU orientation is fused; when `False`, only raw gyro/accel is exposed. <br/> ***bool:*** `False`                                                                          |
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
> To read the arguments for individual packages, add the `-s` flag to the `ros2 launch`
> command (e.g. `ros2 launch husarion_ugv_bringup bringup.launch.yaml -s`).

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
