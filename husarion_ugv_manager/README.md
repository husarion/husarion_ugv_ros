# husarion_ugv_manager

A package containing nodes responsible for high-level control of Husarion UGV.

## Launch Files

This package contains:

- `manager.launch.py`: Responsible for launching behavior trees responsible for safety and LED animations scheduling.

## Configuration Files

- [`lights.xml`](./behavior_trees/lights.xml): BehaviorTree for managing lights.
- [`LightsBT.btproj`](./behavior_trees/LightsBT.btproj): BehaviorTree project for managing Panther lights.
- [`SafetyBT.btproj`](./behavior_trees/SafetyBT.btproj): BehaviorTree project for managing Panther safety protocols.
- [`safety.xml`](./behavior_trees/safety.xml): BehaviorTree for monitoring and managing dangerous situations.
- [`shutdown.xml`](./behavior_trees/shutdown.xml): BehaviorTree for initiating shutdown procedures.
- [`lights_manager_parameters.yaml`](./config/lights_manager_parameters.yaml): Defines parameters for the `lights_manager` node.
- [`lights_manager.yaml`](./config/lights_manager.yaml): Contains parameters for the `lights_manager` node.
- [`safety_manager_parameters.yaml`](./config/safety_manager_parameters.yaml): Defines parameters for the `safety_manager` node.
- [`safety_manager.yaml`](./config/safety_manager.yaml): Contains parameters for the `safety_manager` node.
- [`shutdown_hosts.yaml`](./config/shutdown_hosts.yaml): List with all hosts to request shutdown.

## ROS Nodes

### lights_manager_node

Node responsible for managing Bumper Lights animation scheduling.

#### Subscribers

- `battery/battery_status` [*sensor_msgs/BatteryState*]: State of the internal Battery.
- `hardware/e_stop` [*std_msgs/Bool*]: State of emergency stop.

#### Service Clients (for Default Trees)

- `~/lights/set_animation` [*husarion_ugv_msgs/SetLEDAnimation*]: Allows setting animation on Bumper Lights based on animation ID.

#### Parameters

- `battery.anim_period.critical` [*float*, default: **15.0**]: Time in **[s]** to wait before repeating animation, indicating a critical Battery state.
- `battery.anim_period.low` [*float*, default: **30.0**]: Time in **[s]** to wait before repeating the animation, indicating a low Battery state.
- `battery.charging_anim_step` [*float*, default: **0.1**]: This parameter defines the minimum change in battery percentage required to trigger an update in the battery charging animation.
- `battery.percent.threshold.critical` [*float*, default: **0.1**]: If the Battery percentage drops below this value, an animation indicating a critical Battery state will start being displayed.
- `battery.percent.threshold.low` [*float*, default: **0.4**]: If the Battery percentage drops below this value, the animation indicating a low Battery state will start being displayed.
- `battery.percent.window_len` [*int*, default: **6**]: Moving average window length used to smooth out Battery percentage readings.
- `bt_project_path` [*string*, default: **$(find husarion_ugv_manager)/config/PantherBT.btproj**]: Path to a BehaviorTree project.
- `plugin_libs` [*list*, default: **Empty list**]: List with names of plugins that are used in the BT project.
- `ros_communication_timeout.availability` [*float*, default: **1.0**]: Timeout **[s]** to wait for a service/action while initializing BT node.
- `ros_communication_timeout.response` [*float*, default: **1.0**]: Timeout **[s]** to receive a service/action response after call.
- `ros_plugin_libs` [*list*, default: **Empty list**]: List with names of ROS plugins that are used in a BT project.
- `timer_frequency` [*float*, default: **10.0**]: Frequency **[Hz]** at which lights tree will be ticked.

### safety_manager_node

Node responsible for managing safety features, and software shutdown of components.

#### Subscribers

- `battery/battery_status` [*sensor_msgs/BatteryState*]: State of the internal Battery.
- `hardware/e_stop` [*std_msgs/Bool*]: State of emergency stop.
- `hardware/io_state` [*husarion_ugv_msgs/IOState*]: State of IO pins.
- `hardware/robot_driver_state` [*husarion_ugv_msgs/RobotDriverState*]: State of motor controllers.
- `system_status` [*husarion_ugv_msgs/SystemStatus*]: Built-in computer system status, includes the most important computation-related parameters.

#### Service Clients (for Default Trees)

- `~/hardware/aux_power_enable` [*std_srvs/SetBool*]: Enables Aux Power output.
- `~/hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.
- `~/hardware/fan_enable` [*std_srvs/SetBool*]: Enables fan.

#### Parameters

- `battery.temp.window_len` [*int*, default: **6**]: Moving average window length used to smooth out temperature readings of the Battery.
- `bt_project_path` [*string*, default: **$(find husarion_ugv_manager)/config/PantherBT.btproj**]: Path to a BehaviorTree project.
- `cpu.temp.fan_off` [*float*, default: **60.0**]: Temperature in **[&deg;C]** of the Built-in Computer's CPU, below which the fan is turned off.
- `cpu.temp.fan_on` [*float*, default: **70.0**]: Temperature in **[&deg;C]** of the Built-in Computer's CPU, above which the fan is turned on.
- `cpu.temp.window_len` [*int*, default: **6**]: Moving average window length used to smooth out temperature readings of the Built-in Computer's CPU.
- `driver.temp.fan_off` [*float*, default: **35.0**]: Temperature in **[&deg;C]** of any drivers below which the fan is turned off.
- `driver.temp.fan_on` [*float*, default: **45.0**]: Temperature in **[&deg;C]** of any drivers above which the fan is turned on.
- `driver.temp.window_len` [*int*, default: **6**]: Moving average window length used to smooth out the temperature readings of each driver.
- `fan_turn_off_timeout` [*float*, default: **60.0**]: Minimal time in **[s]**, after which the fan may be turned off.
- `plugin_libs` [*list*, default: **Empty list**]: List with names of plugins that are used in the BT project.
- `ros_communication_timeout.availability` [*float*, default: **1.0**]: Timeout **[s]** to wait for a service/action while initializing BT node.
- `ros_communication_timeout.response` [*float*, default: **1.0**]: Timeout **[s]** to receive a service/action response after call.
- `ros_plugin_libs` [*list*, default: **Empty list**]: List with names of ROS plugins that are used in a BT project.
- `shutdown_hosts_path` [*string*, default: **None**]: Path to a YAML file containing a list of hosts to request shutdown. To correctly format the YAML file, include a **hosts** field consisting of a list with the following fields:
  - `ip` [*string*, default: **None**]: IP of a host to shutdown using HTTP request.
  - `port` [*string*, default: **3003**]: HTTP communication port.
  - `secret` [*string*, default: **husarion**]: Secret to sign HTTP request with.
  - `timeout` [*string*, default: **5.0**]: Time in **[s]** to wait for the host to shutdown. The Built-in Computer will turn off after all computers are shutdown or reached timeout. Keep in mind that hardware will cut power off after a given time after pressing the power button. Refer to the hardware manual for more information.
- `timer_frequency` [*float*, default: **10.0**]: Frequency **[Hz]** at which safety tree will be ticked.
