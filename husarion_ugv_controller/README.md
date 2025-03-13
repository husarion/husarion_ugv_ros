# husarion_ugv_controller

The package contains the default configuration and launch files necessary to start all the basic functionalities of the Husarion UGV.

## Launch Files

- `controller.launch.py`: Establishes communication with the hardware by loading the robot's URDF with plugins and configures the controllers to exchange information between the engine driver and the IMU.

## Configuration Files

- [`WH01_controller.yaml`](./config/WH01_controller.yaml): Configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller` controllers for default WH01 wheels.
- [`WH02_controller.yaml`](./config/WH02_controller.yaml): Configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller` controllers for mecanum WH02 wheels.
- [`WH04_controller.yaml`](./config/WH04_controller.yaml): Configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller` controllers for small pneumatic WH04 wheels.
- [`WH05_controller.yaml`](./config/WH05_controller.yaml): Configures `imu_broadcaster`, `joint_state_broadcaster` and `drive_controller` controllers for Lynx pneumatic wheels.
