# husarion_ugv_localization

The package is responsible for activating mods responsible for fusion of data related to the robot's location.

## Launch files

This package contains:

- `localization.launch.py`: Responsible for activating EKF filtration along with the necessary dependencies needed to operate GPS.
- `nmea_navsat.launch.py`: Responsible for launching the NMEA NavSat driver node.

## Configuration Files

- [`enu_localization.yaml`](./config/enu_localization.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders** and **IMU**. Orientation follows East-North-Up (ENU) coordinates.
- [`enu_localization_with_gps.yaml`](./config/enu_localization_with_gps.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**, and **GPS**. Orientation follows East-North-Up (ENU) coordinates.
- [`nmea_navsat.yaml`](./config/nmea_navsat.yaml): contains parameters for NMEA NavSat driver node.
- [`relative_localization.yaml`](./config/relative_localization.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**. The initial orientation is always 0 in relative mode.
- [`relative_localization_with_gps.yaml`](./config/relative_localization_with_gps.yaml): configures data fusion for `ekf_filter` and `navsat_transform` nodes, using **wheel encoders**, **IMU**, and **GPS**. The initial orientation is always 0 in relative mode.
