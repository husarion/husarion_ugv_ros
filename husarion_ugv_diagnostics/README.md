# husarion_ugv_diagnostics

Package containing nodes monitoring and publishing the Built-in Computer status of Husarion UGV.

## Launch Files

- `system_monitor.launch.py`: Launch a node that analyzes the state of the most important components in the robot

## Configuration Files

- [`system_monitor.yaml`](./config/system_monitor.yaml): Defines parameters for `system_monitor_node`.

## ROS Nodes

### system_monitor_node

Publishes the built-in computer system status, monitoring parameters such as CPU usage, RAM usage, disk usage, and CPU temperature.

#### Publishes

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: System monitor diagnostic messages.
- `system_status` [*husarion_ugv_msgs/SystemStatus*]: Built-in computer system status, includes the most important computation-related parameters.

#### Parameters

- `cpu_usage_warn_threshold` [*float*, default: **95.0**]: Threshold for CPU usage warning in percentage.
- `cpu_temperature_warn_threshold` [*float*, default: **80.0**]: Threshold for CPU temperature warning in degrees Celsius.
- `ram_usage_warn_threshold` [*float*, default: **90.0**]: Threshold for memory usage warning in percentage.
- `disk_usage_warn_threshold` [*float*, default: **90.0**]: Threshold for disk usage warning in percentage.
- `publish_frequency` [*double*, default: **5.0**]: System status publishing frequency [Hz].
