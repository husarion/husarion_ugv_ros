# husarion_ugv_controller

## Changing Velocity Smoothing Parameters

The default drive controller is based on [diff_drive_controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) from [ros2 control](https://control.ros.org/master/index.html) or [mecanum_drive_controller](https://github.com/husarion/husarion_controllers/tree/main/mecanum_drive_controller). This controller can be customized, among others: by modifying the robot's dynamic parameters (e.g. smooth the speed or limit the robot's speed and acceleration). Its parameters can be found in the [husarion_ugv_controller](https://github.com/husarion/husarion_ugv_ros/tree/ros2-devel/husarion_ugv_controller/config). By default, these values correspond to the upper limits of the robot's velocities and accelerations.

## Changing Wheel Type

Changing wheel types is possible and can be done for both the real robot and the simulation. By default, three types of wheels are supported using the launch argument `wheel_type`. If you want to use custom wheels, all you need to do is point to the new wheel and controller configuration files using the `wheel_config_path` and `controller_config_path` parameters. These files should be based on the default files, i.e. [WH01_controller.yaml](./config/WH01_controller.yaml) and [WH01.yaml](../husarion_ugv_description/config/WH01.yaml).
