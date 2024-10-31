# panther_docking

The package contains a `PantherChargingDock` plugin for the [opennav_docking](https://github.com/open-navigation/opennav_docking) project. Thanks to this package, Panther can dock to a charging station.

## Launch Files

- `docking.launch.py`: Launch a node that creates `docking_server` and runs a `PantherChargingDock` plugin. Also it launches `station.launch.py`.
- `station.launch.py`: Launch a node that creates a charging station description with generated apriltag.

## Configuration Files

- [`panther_docking_server.yaml`](./config/panther_docking_server.yaml): Defines parameters for a `docking_server` and a `PantherChargingDock` plugin.

## ROS Nodes

- `DockPosePublisherNode`: A node listens to `tf` and republishes position of `dock_pose` in the fixed frame.
- `PantherChargingDock`:  A plugin for a Panther robot what is responsible for a charger service.

### DockPosePublisherNode

#### Publishes

- `docking/dock_pose` [*geometry_msgs/PoseStamped*]: An offset dock pose.

#### Subscribers

- `tf` [*tf2_msgs/TFMessage*]: Tf tree with a detected dock transform.

#### Parameters

- `fixed_frame` [*string*, default: **odom**]: A fixed frame id of a robot.
- `<dock_name>.type` [*string*, default: **panther_charging_dock**]: It checks if this dock with name `dock_name` is a type of  `panther_charging_dock`.
- `<dock_name>.frame` [*string*, default: **main_wibotic_receiver_requested_pose_link** ]: Then look for transformation between `fixed_frame` and `<dock_name>.frame`  to publish `dock_pose`

### PantherChargingDock

#### Publishes

- `docking/staging_pose` [*geometry_msgs/PoseStamped*]: An offset staging pose next to a charging station.

#### Subscribers

- `docking/dock_pose` [*geometry_msgs/PoseStamped*]: An offset dock pose.

#### Parameters

- `base_frame` [*string*, default: **base_link**]: A base frame id of a robot.
- `fixed_frame` [*string*, default: **odom**]: A fixed frame id of a robot.
- `<dock_type>.external_detection_timeout` [*double*, default: **0.2**]: A timeout in seconds for looking up a transformation from an april tag of a dock to a base frame id.
- `<dock_type>.docking_distance_threshold` [*double*, default: **0.05**]: A threshold of a distance between a robot pose and a dock pose to declare if docking succeed.
- `<dock_type>.docking_yaw_threshold` [*double*, default: **0.3**]: A threshold of a difference of yaw angles between a robot pose and a dock pose to declare if docking succeed.
- `<dock_type>.staging_x_offset` [*double*, default: **-0.7**]: A staging pose is defined by offsetting a dock pose in axis X.
- `<dock_type>.filter_coef` [*double*, default: **0.1**]: A key parameter that influences the trade-off between the filter's responsiveness and its smoothness, balancing how quickly it reacts to new pose data pose how much it smooths out fluctuations.
