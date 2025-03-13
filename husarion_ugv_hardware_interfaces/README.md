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
- `pdo_motor_states_timeout_ms` [*int*, default: **15**]: Depends on the frequency at which Roboteq is configured to send motor states (PDO 1 and 2) data. By default, there should be 10 **[ms]** between received data, if it takes more than `pdo_motor_states_timeout_ms`, a motor states read error is triggered. The default value is set to be expected period +50% margin.
- `pdo_driver_state_timeout_ms` [*int*, default: **75**]: Depends on the frequency at which Roboteq is configured to send driver state (PDO 3 and 4) data. By default, there should be 50 **[ms]** between received data, if it takes more than `pdo_driver_state_timeout_ms`, a driver state read error is triggered. The default value is set to be expected period +50% margin.
- `driver_states_update_frequency` [*float*, default: **20.0**]: As by default, the driver state is published with lower frequency, it also shouldn't be updated with every controller loop iteration. The exact frequency at which driver state is published won't match this value - it will also depend on the frequency of the controller (the exact value of the period can be calculated with the following formula `controller_frequency / ceil(controller_frequency / driver_states_update_frequency)`).
- `max_roboteq_initialization_attempts` [*int*, default: **5**]: In some cases, an SDO error can happen during initialization, it is possible to configure more attempts, before escalating to a general error.
- `max_roboteq_activation_attempts` [*int*, default: **5**]: Similar to initialization, it is possible to allow some SDO errors before escalating to error.
- `max_write_pdo_cmds_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.
- `max_read_pdo_motor_states_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.
- `max_read_pdo_driver_state_errors_count` [*int*, default: **2**]: How many consecutive errors can happen before escalating to general error.

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

Madgwick filter settings

- `use_mag` [*bool*, default: **false**]: Use magnitude to calculate orientation.
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
