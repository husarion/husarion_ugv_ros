# panther_battery

The package containing nodes monitoring and publishing the internal battery state of the Husarion Panther robot.

## Launch Files

This package contains:

- `battery.launch.py`: Responsible for activating battery node, which dealing with reading and publishing battery data.

## ROS Nodes

### battery_driver_node

Publishes battery state read from ADC unit for Panther version 1.2 and above, or based on Roboteq motor controllers' data for earlier.versions of the robot.

#### Publishes

- `_battery/battery_1_status_raw` [*sensor_msgs/BatteryState*]: First battery raw state.
- `_battery/battery_2_status_raw` [*sensor_msgs/BatteryState*]: Second battery raw state. Published if second battery detected.
- `battery/battery_status` [*sensor_msgs/BatteryState*]: Mean values of both batteries if Panther has two batteries. Otherwise, the state of the single battery will be published.
- `battery/charging_status` [*panther_msgs/ChargingStatus*]: Battery charging status.
- `diagnostics` [*diagnostic_msgs/DiagnosticArray*]: Battery diagnostic messages.

#### Subscribers

- `hardware/io_state` [*panther_msgs/IOState*]: Current state of IO.
- `hardware/motor_controllers_state` [*panther_msgs/DriverState*]: Current motor controllers' state and error flags. Subscribed if using Roboteq motor controllers data.

#### Parameters

- `~/adc/device0` [*string*, default: **/sys/bus/iio/devices/iio:device0**]: ADC nr 0 IIO device. Used with Panther version 1.2 and above.
- `~/adc/device1` [*string*, default: **/sys/bus/iio/devices/iio:device1**]: ADC nr 1 IIO device. Used with Panther version 1.2 and above.
- `~/adc/ma_window_len/charge` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery charge readings. Used with Panther version 1.2 and above.
- `~/adc/ma_window_len/temp` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery temperature readings. Used with Panther version 1.2 and above.
- `~/battery_timeout` [*float*, default: **1.0**]: Specifies the timeout in seconds. If the node fails to read battery data exceeding this duration, the node will publish an unknown battery state.
- `~/ma_window_len/voltage` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery voltage readings.
- `~/ma_window_len/current` [*int*, default: **10**]: Window length of a moving average, used to smooth out battery current readings.
- `~/roboteq/driver_state_timeout` [*float*, default: **0.2**]: Specifies timeout in seconds after which driver state messages will be considered old. Used with Panther version 1.06 and earlier.
