# husarion_ugv_battery

The package containing nodes monitoring and publishing the internal battery state of the Husarion UGV.

## Launch Files

This package contains:

- `battery.launch.py`: Responsible for activating battery node, which dealing with reading and publishing battery data.

## Configuration Files

- [`battery_parameters.yaml`](./config/battery_parameters.yaml): Defines parameters for `battery_driver_node`.

## ROS Nodes

### battery_driver_node

Publishes battery state read from ADC unit.

#### Publishes

- `_battery/battery_1_status_raw` [*sensor_msgs/BatteryState*]: First battery raw state.
- `_battery/battery_2_status_raw` [*sensor_msgs/BatteryState*]: Second battery raw state. Published if second battery detected.
- `battery/battery_status` [*sensor_msgs/BatteryState*]: Mean values of both batteries if robot has two batteries. Otherwise, the state of the single battery will be published.
- `battery/charging_status` [*husarion_ugv_msgs/ChargingStatus*]: Battery charging status.
- `battery/protection` [*husarion_ugv_msgs/BatteryProtection*]: Battery undervoltage protection state, see [Battery undervoltage protection](#battery-undervoltage-protection-uvp). Only with `uvp.enable`.
- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Battery diagnostic messages.

#### Subscribers

- `hardware/io_state` [*husarion_ugv_msgs/IOState*]: Current state of IO.
- `hardware/robot_driver_state` [*husarion_ugv_msgs/RobotDriverState*]: Current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data.

#### Parameters

- `adc.device0` [*string*, default: **/sys/bus/iio/devices/iio:device0**]: ADC number 0 IIO device.
- `adc.device1` [*string*, default: **/sys/bus/iio/devices/iio:device1**]: ADC number 1 IIO device.
- `adc.ma_window_len.charge` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery charge readings.
- `adc.ma_window_len.temp` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery temperature readings.
- `battery_timeout` [*float*, default: **1.0**]: Specifies the timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.
- `ma_window_len.voltage` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery voltage readings.
- `ma_window_len.current` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery current readings.
- `roboteq.driver_state_timeout` [*float*, default: **0.2**]: Specifies timeout in seconds after which driver state messages will be considered old (deprecated).
- `uvp.*`: Battery undervoltage protection, see below.

## Battery undervoltage protection (UVP)

The protection makes the robot react before the battery's own BMS cuts the power. It is off by default: with `uvp.enable` false, nothing of it runs and the node behaves exactly as without it. Husarion UGV OS turns it on with the `uvp_enable:=true` launch argument of `bringup.launch.py`, which also enables the parts in the hardware system and the managers.

The battery driver owns the detection and every setting. Each battery is watched on its raw voltage (no moving average). Two thresholds trip after `uvp.consecutive_samples` consecutive valid reads below them:

- **CHARGE_ASAP** at the UVP threshold + `uvp.charge_asap_offset_v`: charge as soon as possible. A WARN in the diagnostics and the CHARGE_ASAP animation of the lights, no limit on driving. Clears when the battery charges.
- **UVP** at the UVP threshold: in `STOP` mode the hardware system latches the e-stop, and the safety manager shuts the robot down after `uvp.stop_timeout_min` without charging. In `SHUTDOWN` mode the safety manager shuts it down at once. UVP stays latched until the battery charges with its raw voltage at least `uvp.clear_hysteresis_v` above the threshold, or the robot reboots. A restart of the driver container does not clear it.

A read is valid when the voltage is between 20 and 50 V and the battery's thermistor reads a temperature - a missing or disconnected pack never trips the protection. With no valid read for longer than `battery_timeout` the diagnostics report the protection as blind. On a Panther each battery is watched on its own and the robot state is the worse of the two. The charging status counts after it held for `uvp.charging_debounce_s`.

The UVP threshold is, in this order: the `uvp.threshold_v` parameter (a permanent WARN in the diagnostics), the `robot.battery.uvp_threshold_v` key of the raw HAT EEPROM YAML at `uvp.hat_eeprom_path`, or 31.0 V. The merged `/run/husarion/robot_config.yaml` is never read, as a root-writable file can override it.

`uvp.stop_timeout_min` set to **998** turns every reaction off, on both thresholds and in both modes: no e-stop, no shutdown, no AUX power off and no CHARGE_ASAP animation. The detection, the topic and a permanent WARN stay - battery protection is then the customer's responsibility. On a robot whose battery data comes from the motor controllers (no ADC), the charging cannot be detected, so the protection only detects and reports. The battery dead shutdown at 27 V is not part of the protection and stays active in every case.

The parameters are read once at start. Every one accepts a number, a bool or the same as text, and a value outside its range is an ERROR in the "Battery protection" diagnostic task and is replaced by the default - never a reason for the node to stop.

- `uvp.enable` [*bool*, default: **false**]: Enables the protection.
- `uvp.threshold_v` [*float*, default: **unset**]: UVP threshold override, 28.0-36.0 V. Empty or NaN is unset.
- `uvp.charge_asap_offset_v` [*float*, default: **0.5**]: CHARGE_ASAP threshold above the UVP threshold, 0.2-3.0 V.
- `uvp.consecutive_samples` [*int*, default: **2**]: Consecutive valid reads below a threshold that trip it, 1-10.
- `uvp.clear_hysteresis_v` [*float*, default: **0.5**]: Voltage above the UVP threshold needed, while charging, to clear UVP, 0.2-2.0 V.
- `uvp.mode` [*string*, default: **STOP**]: `STOP` or `SHUTDOWN`.
- `uvp.stop_timeout_min` [*int*, default: **5**]: T, minutes without charging before the shutdown in `STOP` mode, 1-60, or 998 to turn the reactions off.
- `uvp.aux_off_at_trip` [*bool*, default: **false**]: Turn the AUX power off at the trip in `STOP` mode.
- `uvp.drive_window_s` [*float*, default: **60**]: Seconds the robot may drive after the e-stop is released while UVP is latched, 10-300.
- `uvp.shutdown_hosts_timeout_s` [*float*, default: **10**]: Upper bound of the wait for each user computer in the UVP shutdown, 1-30 s.
- `uvp.charging_debounce_s` [*float*, default: **5**]: Seconds the charging must last before it counts, 1-30.
- `uvp.hat_eeprom_path` [*string*, default: **/proc/device-tree/hat/custom_0**]: Raw HAT EEPROM YAML. The container must be able to read it.
- `uvp.latch_path` [*string*, default: **""**]: File that keeps the UVP latch, its trip record and trip time across driver restarts. Put it on a RAM file system so a reboot clears it. Empty keeps the latch in memory only: after a driver restart the protection trips again within a few reads if the voltage is still low.

Every parameter is also a launch argument of `battery.launch.py` and `bringup.launch.py`, named with `_` instead of `.`: `uvp_enable`, `uvp_threshold_v`, `uvp_charge_asap_offset_v`, `uvp_consecutive_samples`, `uvp_clear_hysteresis_v`, `uvp_mode`, `uvp_stop_timeout_min`, `uvp_aux_off_at_trip`, `uvp_drive_window_s`, `uvp_shutdown_hosts_timeout_s`, `uvp_charging_debounce_s`, `uvp_hat_eeprom_path` and `uvp_latch_path`.

The protection does not run in simulation, which has no battery driver.
