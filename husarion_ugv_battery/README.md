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
