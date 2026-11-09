# husarion_ugv_hardware_interfaces

Package that implements SystemInterface from ros2_control for Husarion UGV.

## ROS Nodes

This package doesn't contain any standalone nodes, only plugins that are loaded by the resource manager. To use this hardware interface, add it to your URDF. You can check how to do it in [panther_macro.urdf.xacro](../husarion_ugv_description/urdf/panther/panther_macro.urdf.xacro) or [lynx_macro.urdf.xacro](../husarion_ugv_description/urdf/lynx/lynx_macro.urdf.xacro).

### UGVSystem (PantherSystem | LynxSystem)

Plugins for Panther and Lynx are based on an abstraction called UGVSystem. Most parts of both systems are similar and are described below. Each plugin is responsible for communicating with engine controllers via the CAN bus, providing E-Stop functionality, and managing the built-in computer's GPIO ports.

#### Publishers

- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: System diagnostic messages.
- `hardware/e_stop` [*std_msgs/Bool*]: Current E-stop state.
- `hardware/io_state` [*husarion_ugv_msgs/IOState*]: Current IO state.
- `hardware/robot_driver_state` [*husarion_ugv_msgs/RobotDriverState*]: Current motor controllers' state and error flags.

#### Diagnostics

The `diagnostics` topic carries three tasks of the `hardware_controller` node:

- `E-stop`: whether the E-stop is latched and what latched it: the driver start (the safety board powers up latched), software (a service call, a communication error, a deactivation or a shutdown), or the hardware line with no software trigger (an e-stop button, a Roboteq fault or the safety board watchdog), plus the last refused reset. Only a latch by the hardware line or by the battery undervoltage protection is a WARN. With `uvp_enable` it also shows the UVP state, the drive window and the age of the protection data.
- `System errors`: the motor controller faults and the latched driver errors. Conditions that do not stop the robot, such as a late PDO, an active current limit or a safety stop with the E-stop released, are WARN. Motor torque disabled on request is a WARN too. While it is off on request, a real emergency stop fault of a motor controller looks the same as the request.
- `System status`: voltages, currents and temperatures of each motor controller, the IO lines, the longest period between two reads since the last report and the PDO timeouts in the last hour.

Earlier versions named the tasks `system errors` and `system status`, keyed the latched driver errors by their internal id (`READ_PDO_MOTOR_STATES error`) and reported the Lynx values as `Default driver ...`. Anything that matches on those names needs updating.

#### Subscribers

- `battery/protection` [*husarion_ugv_msgs/BatteryProtection*]: Battery undervoltage protection state. Only with `uvp_enable`.

#### Service Servers

- `hardware/aux_power_enable` [*std_srvs/SetBool*]: Enables or disables AUX power.
- `hardware/charger_enable` [*std_srvs/SetBool*]: Enables or disables charger.
- `hardware/digital_power_enable` [*std_srvs/SetBool*]: Enables or disables digital power.
- `hardware/e_stop_reset` [*std_srvs/Trigger*]: Resets E-stop.
- `hardware/e_stop_trigger` [*std_srvs/Trigger*]: Triggers E-stop.
- `hardware/fan_enable` [*std_srvs/SetBool*]: Enables or disables fan.
- `hardware/led_control_enable` [*std_srvs/SetBool*]: Enables or disables SBC (Single Board Computer) control over the LEDs.
- `hardware/motor_torque_enable` [*std_srvs/SetBool*]: Allows to enable/disable motor torque when the E-Stop is triggered.

#### Parameters

Required parameters are defined when including the interface in the URDF (you can check out [panther_macro.urdf.xacro](../husarion_ugv_description/urdf/panther/panther_macro.urdf.xacro) or [lynx_macro.urdf.xacro](../husarion_ugv_description/urdf/lynx/lynx_macro.urdf.xacro)).

Physical properties

- `encoder_resolution` [*int*, default: **1600**]: Property of the encoder used, shouldn't be changed.
- `gear_ratio` [*float*, default: **30.08**]: Property of the gearbox used, shouldn't be changed.
- `motor_torque_constant` [*float*, default: **0.11**]: Same as set in the Roboteq driver (TNM parameter), also shouldn't be changed, as it is measured property of the motor.
- `max_rpm_motor_speed` [*float*, default: **3600.0**]: Max RPM speed set in the Roboteq driver (MXRPM parameter).
- `gearbox_efficiency` [*float*, default: **0.75**]: Measured efficiency, used for converting read current to torque, can vary depending on different factors such as temperature and wear.

CAN settings

- `can_interface_name` [*string*, default: **robot_can**]: Name of the CAN interface.
- `master_can_id` [*int*, default: **3**]: CAN ID of the master device (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).
- `sdo_operation_timeout_ms` [*int*, default: **100**]: Timeout of the SDO operations, currently no SDO operation is required in RT operation, so this timeout can be set to a higher value.
- `pdo_motor_states_timeout_ms` [*int*, default: **15**]: Depends on the frequency at which Roboteq is configured to send motor states (PDO 1 and 2) data. By default, there should be 10 **[ms]** between received data, if it takes more than `pdo_motor_states_timeout_ms`, a motor states read error is triggered. The default value is set to be expected period +50% margin. It can be set with the launch argument of the same name.
- `pdo_driver_state_timeout_ms` [*int*, default: **75**]: Depends on the frequency at which Roboteq is configured to send driver state (PDO 3 and 4) data. By default, there should be 50 **[ms]** between received data, if it takes more than `pdo_driver_state_timeout_ms`, a driver state read error is triggered. The default value is set to be expected period +50% margin. It can be set with the launch argument of the same name.
- `driver_states_update_frequency` [*float*, default: **20.0**]: As by default, the driver state is published with lower frequency, it also shouldn't be updated with every controller loop iteration. The exact frequency at which driver state is published won't match this value - it will also depend on the frequency of the controller (the exact value of the period can be calculated with the following formula `controller_frequency / ceil(controller_frequency / driver_states_update_frequency)`).
- `max_roboteq_initialization_attempts` [*int*, default: **5**]: In some cases, an SDO error can happen during initialization, it is possible to configure more attempts, before escalating to a general error.
- `max_roboteq_activation_attempts` [*int*, default: **5**]: Similar to initialization, it is possible to allow some SDO errors before escalating to error.
- `max_write_pdo_cmds_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.
- `max_read_pdo_motor_states_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.
- `max_read_pdo_driver_state_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.
- `limit_tpdo_rates` [*bool*, default: **false**]: Use the master configuration from [limited_tpdo_rates](./config/limited_tpdo_rates/canopen_configuration.yaml), which writes TPDO inhibit and event timers into the Roboteq on every boot: motor states at 200-250 Hz, battery and temperatures at 20-25 Hz. Set it with the `limit_tpdo_rates` launch argument. The Roboteq keeps the timers in RAM until it is power cycled, so turning the parameter off again takes a power cycle of the motor controllers. The driver writes a copy of that master configuration to the temporary directory (`/tmp` unless `TMPDIR` is set) at configure, so it needs a writable one.
- `canopen_resync_watchdog` [*bool*, default: **false**]: Exit the process when the CANopen master stops working, see [CANopen resync watchdog](#canopen-resync-watchdog). Set it with the `canopen_resync_watchdog` launch argument.
- `canopen_resync_watchdog_timeout_s` [*int*, default: **30**]: Seconds of continuous motor state PDO timeout after which `canopen_resync_watchdog` exits. Set it with the launch argument of the same name.

#### CANopen resync watchdog

The CANopen master can end up in a state where it no longer receives while the bus is healthy, and only a new initialization heals it. With `canopen_resync_watchdog` the process exits with code 74 (`EX_IOERR`) after `canopen_resync_watchdog_timeout_s` of continuous motor state PDO timeout while the motor controllers' heartbeats keep arriving, or when 3 SDO write confirmations of the same motor controller miss their deadline within a minute of the first miss (a fixed window, not a sliding one). The SDO half also works during configure, deactivate and shutdown, so a stuck master there exits with 74 instead of returning an error.

When to enable it: only under a restart policy, for example `restart: always` in docker compose or `Restart=always` in systemd. The exit stops the whole `controller_manager` process, including any other hardware component and controller loaded in it. Under `ros2 launch` the exit of `ros2_control_node` shuts the whole launch down, and the launch does not pass the 74 on, so an on-failure policy may not restart it: use an always-restart policy. Motor controllers that lose power (main switch stage 1 on Panther, a battery cut, an e-stop circuit that cuts their logic power) stop their heartbeats, and a heartbeat timeout or a CAN error does not count, so they do not cause a restart loop. A latched e-stop does not trigger it either: the motor state PDOs keep coming while it is latched.

Before the PDO exit the driver tries for at most 100 ms to latch the motor controllers' e-stop, which only helps if the master can still send. Otherwise the exit sends no NMT stop and no zero command. On Panther the GPIO watchdog stops with the process and the safety board latches the e-stop. On Lynx the wheels are stopped only by the Roboteq's own command watchdog (`RWD`), which this driver does not configure: check that it is set in your motor controller configuration before you enable this on a Lynx. The driver warns about it at startup. The PDO exit is logged through ROS and the SDO exit on stderr. The process exits without flushing the ROS log files, so the last line may only be in the console output.

Real-time thread priorities. Each is a SCHED_FIFO priority from 1 to 98. For the threads that accept 0, it means the thread sets no priority of its own and keeps the one it inherited. Each can be set with the launch argument of the same name. A missing parameter keeps the default, and an invalid value is replaced by the default with a warning. Each thread also carries a name, so it can be found with `ps -eLo tid,rtprio,psr,comm` and pinned to a CPU from outside the process.

- `configure_thread_priority` [*int*, default: **0**]: Priority the thread that runs `on_configure` takes when the CANopen manager initializes. Every thread it creates afterwards inherits it: the threads below before they set their own priority, the ROS interface executor that serves `hardware/e_stop_trigger` and `hardware/e_stop_reset`, and other helper threads. 0 leaves that thread as it is, so the helper threads run SCHED_OTHER and only the threads below run RT.
- `canopen_thread_priority` [*int*, default: **70**]: CANopen communication thread (`lely-io`). It reads the CAN frames and has to stay above the control loop (FIFO 50, or `thread_priority` in the controller config), or the motor state timestamps go stale and the driver latches the e-stop. 0 is not accepted, and a value of 50 or less (the default control loop priority) logs a warning.
- `rpdo_dispatch_priority` [*int*, default: **65**]: RPDO dispatch thread of each Roboteq driver (`rpdo-dispatch`), which timestamps every motor state frame and runs the boot callbacks.
- `gpio_watchdog_priority` [*int*, default: **60**]: Safety watchdog thread (`gpio-wdog`). It toggles the watchdog pin every 10 ms while the e-stop is released, and the safety board cuts motor power when the toggling stops. A trigger waits for this thread to stop the toggling, as it did on earlier versions, so a thread kept off the CPU delays the trigger and it has to stay above the control loop too. 0 is not accepted, and a value of 50 or less (the default control loop priority) logs a warning.
- `gpio_monitor_priority` [*int*, default: **60**]: GPIO edge event monitor thread (`gpio-monit`).

Real-time thread CPUs. Each thread pins itself to the given CPU when it starts, and -1 leaves it unpinned. A value that is not -1 or a CPU of this machine leaves the thread unpinned with a warning, and so does a failed pin. All of them are unpinned by default and set from the launch arguments of the same name.

- `canopen_thread_cpu` [*int*, default: **-1**]: CPU of the CANopen communication thread.
- `rpdo_dispatch_cpu` [*int*, default: **-1**]: CPU of the RPDO dispatch threads.
- `gpio_watchdog_cpu` [*int*, default: **-1**]: CPU of the safety watchdog thread. The thread is created once when the GPIO controller starts and parks while the e-stop is latched, so it keeps its CPU and priority across e-stop resets.
- `gpio_monitor_cpu` [*int*, default: **-1**]: CPU of the GPIO monitor thread.

Battery undervoltage protection

- `uvp_enable` [*bool*, default: **unset**]: Present only when the protection is enabled (the `uvp_enable` launch argument), so the robot description is unchanged without it. In `STOP` mode the system then latches the e-stop at a UVP trip reported on `battery/protection`, and after any release of the e-stop while UVP is latched it latches it again when the drive window ends. It never releases the e-stop itself. The "E-stop" diagnostic task shows UVP as the cause, the drive window and an ERROR when the protection data is stale. See [Battery Undervoltage Protection](../husarion_ugv_manager/CONFIGURATION.md#battery-undervoltage-protection-uvp).

PantherSystem additional CAN settings

- `front_driver_can_id` [*int*, default: **1**]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).
- `rear_driver_can_id` [*int*, default: **2**, **Required only for PantherSystem**]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).

LynxSystem additional CAN settings

- `driver_can_id` [*int*, default: **1**]: CAN ID defined in the properties of Roboteq (set as in [canopen_configuration.yaml](./config/canopen_configuration.yaml)).

> [!CAUTION]
> `max_write_pdo_cmds_errors_count`, `max_read_pdo_motor_states_errors_count`, `max_read_pdo_driver_state_errors_count`, `sdo_operation_timeout`, `pdo_motor_states_timeout_ms` and `pdo_driver_state_timeout_ms` are safety-critical parameters, they should be changed only in very specific cases, be sure that you know how they work and be really cautious when changing them.

### PhidgetImuSensor

Plugin responsible for communicating with IMU and filtering data using Madgwick Filter.

#### Parameters

Required parameters are defined when including the interface in the URDF (you can check out [panther_macro.urdf.xacro](../husarion_ugv_description/urdf/panther/panther_macro.urdf.xacro) or [lynx_macro.urdf.xacro](../husarion_ugv_description/urdf/lynx/lynx_macro.urdf.xacro)).

Physical properties

- `serial` [*int*, default: **-1**]: The serial number of the Phidgets Spatial to connect to. If -1 (the default), connects to any Spatial Phidget that can be found.
- `hub_port` [*int*, default: **0**]: The Phidgets VINT hub port to connect to. Only used if the Spatial Phidget is connected to a VINT hub. Defaults to 0.
- `heating_enabled` [*bool*, default: **false**]: Use the internal heating element; just available on MOT0109 onwards. Do not set this parameter for older versions.
- `time_resynchronization_interval_ms` [*int*, default: **5000**]: The number of milliseconds to wait between resynchronizing the time on the Phidgets Spatial with the local time. Larger values have less 'jumps', but will have more timestamp drift. Setting this to 0 disables resynchronization. Defaults to 5000 ms.
- `data_interval_ms` [*int*, default: **8**]: The number of milliseconds between acquisitions of data on the device (allowed values are dependent on the device). Defaults to 8 ms.
- `callback_delta_epsilon_ms` [*int*, default: **1**]: The number of milliseconds epsilon allowed between callbacks when attempting to resynchronize the time. If this is set to 1, then a difference of data_interval_ms plus or minus 1 millisecond will be considered viable for resynchronization. Higher values give the code more leeway to resynchronize, at the cost of potentially getting bad resynchronizations sometimes. Lower values can give better results, but can also result in never resynchronizing. Must be less than data_interval_ms. Defaults to 1 ms.
- `cc_mag_field` [*double*, default: **0.0**]: Ambient magnetic field calibration value; see device's user guide for information on how to calibrate.
- `cc_offset0` [*double*, default: **0.0**]: Calibration offset value 0; see device's user guide for information on how to calibrate.
- `cc_offset1` [*double*, default: **0.0**]: Calibration offset value 1; see device's user guide for information on how to calibrate.
- `cc_offset2` [*double*, default: **0.0**]: Calibration offset value 2; see device's user guide for information on how to calibrate.
- `cc_gain0` [*double*, default: **0.0**]: Gain offset value 0; see device's user guide for information on how to calibrate.
- `cc_gain1` [*double*, default: **0.0**]: Gain offset value 1; see device's user guide for information on how to calibrate.
- `cc_gain2` [*double*, default: **0.0**]: Gain offset value 2; see device's user guide for information on how to calibrate.
- `cc_t0` [*double*, default: **0.0**]: T offset value 0; see device's user guide for information on how to calibrate.
- `cc_t1` [*double*, default: **0.0**]: T offset value 1; see device's user guide for information on how to calibrate.
- `cc_t2` [*double*, default: **0.0**]: T offset value 2; see device's user guide for information on how to calibrate.
- `cc_t3` [*double*, default: **0.0**]: T offset value 3; see device's user guide for information on how to calibrate.
- `cc_t4` [*double*, default: **0.0**]: T offset value 4; see device's user guide for information on how to calibrate.
- `cc_t5` [*double*, default: **0.0**]: T offset value 5; see device's user guide for information on how to calibrate.

Additional parameters

- `use_madgwick_filter` [*bool*, default: **false**]: Whether to calculate orientation using the Madgwick filter. When *true*, orientation is computed and published. When *false* (the default), the orientation in the IMU topic will contain NaNs and the first element of the orientation covariance matrix will be set to -1, according to the [documentation](https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html).

Madgwick filter settings

- `use_mag` [*bool*, default: **false**]: Use the magnetometer to calculate orientation. Set from the `use_mag` launch argument (default **true** in the URDF).
- `gain` [*double*, default: **0.1**]: Gain of the filter. Higher values lead to faster convergence but more noise. Lower values lead to slower convergence but smoother signal.
- `zeta` [*double*, default: **0.1**]: Gyro drift gain (approx. rad/s).
- `mag_bias_x` [*double*, default: **0.0**]: Magnetometer bias (hard iron correction), x component.
- `mag_bias_y` [*double*, default: **0.0**]: Magnetometer bias (hard iron correction), y component.
- `mag_bias_z` [*double*, default: **0.0**]: Magnetometer bias (hard iron correction), z component.
- `stateless` [*bool*, default: **false**]: Use stateless to compute orientation on every data callback without prediction based on previous measurements.
- `remove_gravity_vector` [*bool*, default: **false**]: The gravity vector is kept in the IMU message.

The Madgwick Filter parameters, zeta and gain, are provided in the paper "[An efficient orientation filter for inertial and inertial/magnetic sensor arrays](https://x-io.co.uk/downloads/madgwick_internal_report.pdf)". The Panther robot utilizes the `PhidgetSpatial Precision 3/3/3` IMU sensor, with detailed specifications available on  [the producer's website](https://www.phidgets.com/?prodid=1205#Tab_Specifications).

Gyroscope Noise (@ 1ms) is:

```text
gyroMeasError = 0.2 deg/s = 0.00351 rad/s
```

Gyroscope Drift Max is:

```text
gyroMeasDrift  = 0.1 deg/s = 0.00175 rad/s
```

Instructions for computing gain and zeta can be found in [the paper](https://x-io.co.uk/downloads/madgwick_internal_report.pdf).

```text
gain = sqrt(3/4)* gyroMeasError = 0.00303
```

```text
zeta = sqrt(3/4)* gyroMeasDrift = 0.00151
```

## Code structure

The code structure is described in more detail in a [separate file](./CODE_STRUCTURE.md).

## Generating CAN config

Adjust your configuration and generate a new `master.dcf` using:
`dcfgen canopen_configuration.yaml -r`

### RT

To configure RT check out the instructions provided in the [ros2_control docs](https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html#determinism) (add group and change `/etc/security/limits.conf`).

## Testing

### Setup

Since some of the tests create and access a virtual CAN interface, it is necessary to install `kmod` and `iproute2`.

```bash
sudo apt update
sudo apt install -y kmod iproute2
```

### Running tests

```bash
colcon build --packages-select husarion_ugv_hardware_interfaces --symlink-install
colcon test --event-handlers console_direct+ --packages-select husarion_ugv_hardware_interfaces --parallel-workers 1
colcon test-result --verbose --all
```

As some of the tests are accessing the virtual CAN interface, they can't be executed in parallel (that's why `--parallel-workers 1` flag).
