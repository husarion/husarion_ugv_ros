# husarion_ugv_manager

## Shutdown Behavior

For more information regarding shutdown behavior, refer to `ShutdownHostsFromFile` BT node in the [Actions](#actions) section. An example of a shutdown hosts YAML file can be found below.

``` yaml
# My shutdown_hosts.yaml
hosts:
  # Intel NUC, user computer
  - ip: 10.15.20.3
    port: 3003
  # Universal robots UR5
  - ip: 10.15.20.4
    port: 3003
  # My device that requires very long shutdown sequence
  - ip: 10.15.20.12
    port: 3003
    secret: password123
    timeout: 40
```

> [!IMPORTANT]
>
> To allow your computer to be safe shutdown from Built-in Computer, you need to set up a HTTP server capable of turning off your device. This can be done using snap:
>
> ``` bash
> sudo snap install husarion-shutdown
> sudo snap set husarion-shutdown config.user-computer-ip="10.15.20.12" config.password="password123"
> sudo husarion-shutdown.start
> ```

## Faults Handle

After receiving a message on the `battery/battery_status` topic, the `husarion_ugv_manager` node makes decisions regarding safety measures. For more information regarding the power supply status, please refer to the [BatteryState](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html) message definition and [adc_battery.cpp](../husarion_ugv_battery/src/battery/adc_battery.cpp) implementation.

| Power Supply Health | Procedure                                                                                                                                                                                                                     |
| ------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| GOOD                | -                                                                                                                                                                                                                             |
| UNKNOWN             | -                                                                                                                                                                                                                             |
| OVERHEAT            | 1. Turn on the fan. <br/> 2. If the Battery temperature is higher than 55.0 **[&deg;C]**, trigger an emergency stop and turn off AUX. <br/> 3. If the Battery temperature is higher than 62.0 **[&deg;C]**, shutdown the robot. |
| DEAD                | Shutdown the robot.                                                                                                                                                                                                           |
| OVERVOLTAGE         | 1. Initiate an emergency stop. <br/> 2. Display an error animation if the charger is connected.                                                                                                                                |
| COLD                | -                                                                                                                                                                                                                             |

> [!NOTE]
>
> 1. The fan exhibits a form of hysteresis, allowing it to be turned off after a delay of at least 60 seconds.
> 2. Once the Panther ROS stack initializes, the fan activates and operates for a duration of approximately 60 seconds.

## Battery Undervoltage Protection (UVP)

The battery undervoltage protection makes the robot react before the battery's BMS cuts the power. It is off by default and enabled with the `uvp_enable:=true` launch argument of `bringup.launch.py`. The battery driver detects it and owns every setting - see [husarion_ugv_battery](../husarion_ugv_battery/README.md#battery-undervoltage-protection-uvp) for the thresholds, the detection and the parameters. The reactions are split so that each works on its own:

| Part | Reaction |
| ---- | -------- |
| Hardware system (`ros2_control`) | In `STOP` mode latches the e-stop at the UVP trip and holds it. Works with `disable_manager:=true`. |
| Safety manager | Optional AUX power off at the trip, the stop timeout T and the UVP shutdown. |
| Lights manager | The CHARGE_ASAP animation. |

**STOP mode** (default): at the UVP trip the e-stop latches. The e-stop can be reset with `hardware/e_stop_reset` as usual, with all its checks - the protection never releases it itself. Whenever the e-stop is released while UVP is latched, whatever released it, a drive window of `uvp_drive_window_s` (60 s) opens so the robot can reach a charger, and at its end the e-stop latches again. Every later release opens a new window. If the robot does not charge within `uvp_stop_timeout_min` (T, 5 min) from the trip, it shuts down. Charging cancels T. UVP itself stays latched until the battery charges with its voltage back above the threshold plus the hysteresis, or the robot reboots. If the charging stops while UVP is still latched, a full T starts again. The battery driver counts T, from the trip or from the end of the last charge, so a restart of the safety manager neither extends nor shortens it. With `uvp_aux_off_at_trip` the AUX power goes off at the trip, as soon as the hardware system offers `hardware/aux_power_enable`, and again when the safety manager restarts while UVP is latched, also if it was switched back on in between. The AUX power is cut without a shutdown of what it feeds, so enable it only when no user computer runs from it.

**SHUTDOWN mode**: the robot shuts down at the UVP trip, without an e-stop, once it has not charged for 10 s. While the robot charges the shutdown waits, as T does in `STOP` mode, and it follows 10 s after the charging stops with UVP still latched. A charger counts from its first read, before the battery driver's `uvp_charging_debounce_s` passed, so a robot that the charger powers on does not trip and shut down in a loop, and a short drop of the charging status does not shut it down. A charger that reads connected but never charges holds the shutdown for at most 60 s, twice the longest `uvp_charging_debounce_s`, so a broken charger contact cannot keep the robot running into the BMS cut.

**Stale data.** The protection acts on the last `battery/protection` message, and the battery driver publishes it at least once a second. When it stops, the last state holds: a latched UVP stays latched, an OK never trips. Charging no longer holds the shutdown once the data is over 3 s old, as nothing confirms it any more: T, or the 10 s of `SHUTDOWN` mode, then counts from the last message that reported charging, and the "Battery protection reaction" diagnostic task reports an ERROR. The lights manager stops the CHARGE_ASAP animation on stale data.

**`uvp_reactions_enabled:=false`** turns every reaction off, on both thresholds and in both modes - no e-stop, no shutdown, no AUX power off and no CHARGE_ASAP animation. Detection, the `battery/protection` topic and a permanent WARN in the diagnostics stay. `uvp_stop_timeout_min:=998` does the same; the value is kept for compatibility with existing configurations. On a robot whose battery data comes from the motor controllers (no ADC), charging cannot be detected, so the reactions are always off there.

The UVP shutdown sequence is: the user computers from `shutdown_hosts.yaml`, each waited for at most `uvp_shutdown_hosts_timeout_s` (10 s) or its own `timeout` if shorter, then the AUX power off, then the `Shutdown` tree of the BehaviorTree project as in any other shutdown, a custom one included, which powers the Built-in Computer off. With the shipped tree, the user computers that are already down are skipped there; one that did not shut down within the capped wait gets its own `timeout` again. While UVP is latched, every shutdown takes this sequence, including the one at a dead battery and the power button. The UVP part is built into the safety manager, so a custom `safety_bt_project_path` does not have to provide it. It needs the `shutdown_hosts_from_file_bt_node` and `call_set_bool_service_bt_node` plugins: without them only the `Shutdown` tree runs and the diagnostics report an ERROR.

> [!NOTE]
>
> The wait for the user computers in a shutdown that is not a UVP shutdown is the per-host `timeout` in [`shutdown_hosts.yaml`](./config/shutdown_hosts.yaml), 10 s in the shipped file. It is not a UVP setting.

The battery dead shutdown (battery health DEAD, below 27 V for over 2 s) is not part of the protection and stays active in every mode, also with the reactions off.

### Behaviour with both EMU settings

The EMU is set with DIP switches and the software cannot read its setting. By default an e-stop puts the motor controllers into active braking.

| Mode | EMU default (e-stop = active braking) | EMU set to cut the power at an e-stop |
| ---- | ------------------------------------- | -------------------------------------- |
| STOP | The robot stops, the electronics stay on. There is time to drive to a charger after an e-stop reset, or to plug the charger in. | The power is cut at once, without a shutdown of the OS. |
| SHUTDOWN | A clean shutdown of the system. | Risk of a power cut during the shutdown: stopping the driver triggers the e-stop (the hardware system's `on_shutdown`), which with this EMU setting cuts the power. |

In both cases the robot reacts before the BMS. The settings only decide whether the OS shuts down cleanly.

## BehaviorTree

This package contains two main BehaviorTree projects. One is designed for lights handling and the other for safety and system shutdown.
For a BehaviorTree project to work correctly, it must contain a tree with correct names. The names are: `Lights` for lights BT project; `Shutdown` and `Safety` for safety BT project. Files with trees XML descriptions can be shared between projects. Each tree is provided with a set of default blackboard entries (described below), which can be used to specify the behavior of a given tree.

### Nodes

#### Actions

- `CallSetBoolService` - allows calling the standard **std_srvs/SetBool** ROS service. Provided ports are:
  - `data` [*input*, *bool*, default: **None**]: service data - **true** or **false** value.
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
- `CallSetLedAnimationService` - allows calling custom type **husarion_ugv_msgs/SetLEDAnimation** ROS service. The provided ports are:
  - `id` [*input*, *unsigned*, default: **None**]: animation ID.
  - `param` [*input*, *string*, default: **None**]: optional parameter passed to animation.
  - `repeating` [*input*, *bool*, default: **false**]: indicates if the animation should repeat.
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
- `CallTriggerService` - allows calling the standard **std_srvs/Trigger** ROS service. The provided ports are:
  - `service_name` [*input*, *string*, default: **None**]: ROS service name.
- `ShutdownHostsFromFile` - allows to shutdown devices based on a YAML file. Returns `SUCCESS` only when a YAML file is valid and the shutdown of all defined hosts was successful. Nodes are processed in a semi-parallel fashion. Every tick of the tree updates the state of a host. This allows some hosts to wait for a HTTP server response, while others are already pinged and awaiting a full shutdown. If a host is shutdown, it is no longer processed. In the case of a long timeout is used for a given host, other hosts will be processed simultaneously. The provided ports are:
  - `shutdown_host_file` [*input*, *string*, default: **None**]: global path to YAML file with hosts to shutdown.
  - `timeout_override` [*input*, *float*, default: **None**]: optional upper bound in **[s]** of each host's `timeout` from the file. Unset, zero or negative keeps the timeouts from the file.
- `ExecuteCommand` - allows to execute system command. Will return `SUCCESS` if command was executed successfully. The provided ports are:
  - `command` [*input*, *string*, default: **None**]: command to execute.
  - `timeout` [*input*, *string*, default: **None**]: time in **[s]** to wait for command execution. If this timeout is reached the process executing the command will be killed.
- `SignalShutdown` - signals shutdown of the robot. The provided ports are:
  - `message` [*input*, *string*, default: **None**]: message with reason for robot to shutdown.

#### Decorators

- `TickAfterTimeout` - will skip a child until the specified time has passed. It can be used to specify the frequency at which a node or subtree is triggered. The provided ports are:
  - `timeout` [*input*, *unsigned*, default: **None**]: time in **[s]** to wait before ticking child again.

### Trees

#### Lights

A tree responsible for scheduling animations displayed on the Bumper Lights based on the Husarion Panther robot's system state.

<p align="center">
  <img align="center" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/lights_tree.svg" alt="Lights Behavior Tree"/>
</p>

Default blackboard entries:

- `battery_percent` [*float*, default: **None**]: moving average of the Battery percentage.
- `battery_percent_round` [*string*, default: **None**] Battery percentage rounded to a value specified with `~lights/update_charging_anim_step` parameter and cast to string.
- `battery_health` [*unsigned*, default: **None**]: the current Battery health state.
- `battery_status` [*unsigned*, default: **None**]: the current Battery status.
- `charging_anim_percent` [*string*, default: **None**]: the charging animation Battery percentage value, cast to a string.
- `current_anim_id` [*int*, default: **-1**]: ID of currently displayed state animation.
- `current_battery_anim_id` [*int*, default: **-1**]: ID of currently displayed battery animation.
- `current_error_anim_id` [*int*, default: **-1**]: ID of currently displayed error animation.
- `e_stop_state` [*bool*, default: **None**]: state of E-stop.

Default constant blackboard entries:

- `BATTERY_STATE_ANIM_PERIOD` [*float*, default: **120.0**]: refers to `battery_state_anim_period` ROS parameter.
- `CRITICAL_BATTERY_THRESHOLD_PERCENT` [*float*, default: **0.1**]: refers to `critical_battery_threshold_percent` ROS parameter.
- `LOW_BATTERY_ANIM_PERIOD` [*float*, default: **30.0**]: refers to `low_battery_anim_period` ROS parameter.
- `LOW_BATTERY_THRESHOLD_PERCENT` [*float*, default: **0.4**]: refers to `low_battery_threshold_percent` ROS parameter.
- `E_STOP_ANIM_ID` [*unsigned*, value: **0**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::E_STOP`.
- `READY_ANIM_ID` [*unsigned*, value: **1**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::READY`.
- `ERROR_ANIM_ID` [*unsigned*, value: **2**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::ERROR`.
- `NO_ERROR_ANIM_ID` [*unsigned*, value: **3**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::NO_ERROR`.
- `MANUAL_ACTION_ANIM_ID` [*unsigned*, value: **4**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::MANUAL_ACTION`.
- `LOW_BATTERY_ANIM_ID` [*unsigned*, value: **5**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::LOW_BATTERY`.
- `CRITICAL_BATTERY_ANIM_ID` [*unsigned*, value: **6**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::CRITICAL_BATTERY`.
- `CHARGING_BATTERY_ANIM_ID` [*unsigned*, value: **7**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::CHARGING_BATTERY`.
- `BATTERY_CHARGED_ANIM_ID` [*unsigned*, value: **8**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::BATTERY_CHARGED`.
- `CHARGER_INSERTED_ANIM_ID` [*unsigned*, value: **9**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::CHARGER_INSERTED`.
- `BATTERY_NOMINAL_ANIM_ID` [*unsigned*, value: **10**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::BATTERY_NOMINAL`.
- `AUTONOMOUS_READY_ANIM_ID` [*unsigned*, value: **11**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::AUTONOMOUS_READY`.
- `AUTONOMOUS_ACTION_ANIM_ID` [*unsigned*, value: **12**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::AUTONOMOUS_ACTION`.
- `GOAL_ACHIEVED_ANIM_ID` [*unsigned*, value: **13**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::GOAL_ACHIEVED`.
- `BLINKER_LEFT_ANIM_ID` [*unsigned*, value: **14**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::BLINKER_LEFT`.
- `BLINKER_RIGHT_ANIM_ID` [*unsigned*, value: **15**]: animation ID constant obtained from `husarion_ugv_msgs::LEDAnimation::BLINKER_RIGHT`.
- `POWER_SUPPLY_STATUS_UNKNOWN` [*unsigned*, value: **0**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN`.
- `POWER_SUPPLY_STATUS_CHARGING` [*unsigned*, value: **1**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING`.
- `POWER_SUPPLY_STATUS_DISCHARGING` [*unsigned*, value: **2**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING`.
- `POWER_SUPPLY_STATUS_NOT_CHARGING` [*unsigned*, value: **3**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING`.
- `POWER_SUPPLY_STATUS_FULL` [*unsigned*, value: **4**]: power supply status constant obtained from `sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_FULL`.

### Safety

A tree responsible for monitoring the Panther robot's state and handling safety measures, such as cooling the robot in case of high Built-in Computer's CPU or Battery temperatures.

<!-- TODO: Update tree image (remove timeouts from leafs) -->
<p align="center">
  <img align="center" src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/safety_tree.svg" alt="Safety Behavior Tree"/>
</p>

Default blackboard entries:

- `aux_state` [*bool*, default: **None**]: state of AUX Power.
- `bat_temp` [*double*, default: **None**]: moving average of the Battery temperature.
- `cpu_temp` [*double*, default: **None**]: moving average of the Built-in Computer's CPU temperature
- `driver_temp` [*double*, default: **None**]: moving average of driver temperature. Out of the two drivers, the one with the higher temperature is taken into account.
- `e_stop_state` [*bool*, default: **None**]: state of the E-stop.
- `fan_state` [*bool*, default: **None**]: state of the fan.

Default constant blackboard entries:

- `CPU_FAN_OFF_TEMP` [*float*, default: **60.0**]: refers to the`cpu_fan_off_temp` ROS parameter.
- `CPU_FAN_ON_TEMP` [*float*, default: **70.0**]: refers to the `cpu_fan_on_temp` ROS parameter.
- `CRITICAL_BAT_TEMP` [*float*, default: **59.0**]: refers to the `critical_bat_temp` ROS parameter.
- `DRIVER_FAN_OFF_TEMP` [*float*, default: **35.0**]: refers to the `driver_fan_off_temp` ROS parameter.
- `DRIVER_FAN_ON_TEMP` [*float*, default: **45.0**]: refers to the `driver_fan_on_temp` ROS parameter.
- `HIGH_BAT_TEMP` [*float*, default: **55.0**]: refers to the `high_bat_temp` ROS parameter.
- `POWER_SUPPLY_HEALTH_UNKNOWN` [*unsigned*, value: **0**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN`.
- `POWER_SUPPLY_HEALTH_GOOD` [*unsigned*, value: **1**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD`.
- `POWER_SUPPLY_HEALTH_OVERHEAT` [*unsigned*, value: **2**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT`.
- `POWER_SUPPLY_HEALTH_DEAD` [*unsigned*, value: **3**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_DEAD`.
- `POWER_SUPPLY_HEALTH_OVERVOLTAGE` [*unsigned*, value: **4**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE`.
- `POWER_SUPPLY_HEALTH_UNSPEC_FAILURE` [*unsigned*, value: **5**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE`.
- `POWER_SUPPLY_HEALTH_COLD` [*unsigned*, value: **6**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_COLD`.
- `POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE` [*unsigned*, value: **7**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE`.
- `POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE` [*unsigned*, value: **8**]: power supply status constant obtained from the `sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE`.

### Shutdown

A tree responsible for the graceful shutdown of robot components, user computers, and the Built-in Computer. By default, it will proceed to shutdown all computers defined in a YAML file with a path defined by the `shutdown_host_path` ROS parameter.

<!-- TODO: Update tree image (remove timeouts from leafs) -->
<p align="center">
  <img src="https://github-readme-figures.s3.eu-central-1.amazonaws.com/panther/husarion_ugv/shutdown_tree.svg" alt="Shutdown Behavior Tree"/>
</p>

Default constant blackboard entries:

- `SHUTDOWN_HOSTS_PATH` [*string*, default: **None**]: refers to `shutdown_hosts_path` ROS parameter.

Expected blackboard entries:

- `signal_shutdown` [*pair(bool, string)*, default: **(false, '')**]: flag to shutdown robot with information to display while shutting down.

### Modifying Behavior Trees

Each behavior tree can be easily customized to enhance its functions and capabilities. To achieve this, we recommend using Groot2, a powerful tool for developing and modifying behavior trees. To install Groot2 and learn how to use it, please refer to the [official guidelines](https://www.behaviortree.dev/groot).

When creating a new BehaviorTree project, it is advised to use an existing project as a guideline and leverage it for reference. You can study the structure and implementation of the behavior trees in the existing project to inform your own development process. The project should consist of `Lights` behavior tree or both `Safety` and `Shutdown` behavior tree. Additionally, you have the option to incorporate some of the files used in the existing project into your own project. By utilizing these files, you can benefit from the work already done and save time and effort in developing certain aspects of the behavior trees.

> [!NOTE]
> It is essential to exercise caution when modifying the trees responsible for safety or shutdown and ensure that default behaviors are not removed.
>
> Remember to use the files from the existing project in a way that avoids conflicts, such as by saving them under new names to ensure they don't overwrite any existing files.

When modifying behavior trees, you have the flexibility to use standard BehaviorTree.CPP nodes or leverage nodes created specifically for Panther, as detailed in the [Nodes](#nodes) section. Additionally, if you have more specific requirements, you can even create your own custom Behavior Tree nodes. However, this will involve modifying the package and rebuilding the project accordingly.

To use your customized project, you can modify the `bt_project_file` ROS parameter.

### Real-time Visualization

Groot2 also provides a real-time visualization tool that allows you to see and debug actively running trees. To use this tool with trees launched with the `husarion_ugv_manager` package, you need to specify the port associated with the tree you want to visualize. The ports for each tree are listed below:

- Lights tree: `10.15.20.2:5555`
- Safety tree: `10.15.20.2:6666`
- Shutdown tree: `10.15.20.2:7777`
