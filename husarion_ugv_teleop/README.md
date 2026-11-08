# husarion_ugv_teleop

The package contains configuration and launch files necessary to control the robot via gamepad.

## Launch Files

- `teleop.launch.py`: Enables robot teleoperation via gamepad. Starts `joy2twist_node` from [joy2twist](https://github.com/husarion/joy2twist) and `joy_node`, which reads the gamepad.

### Launch Arguments

- `joy_dev` [*string*, default: **/dev/input/js0**]: Path to the joystick device file used by `joy_node`. If the gamepad is not detected under the default path, check available devices with `ls /dev/input | grep js` and pass the correct one, e.g. `joy_dev:=/dev/input/js1`.

## Configuration Files

- `joy2twist_${ROBOT_MODEL_NAME}.yaml`: Describes button mappings of a gamepad and parameters such as linear and angular velocity.

## ROS Nodes

### joy_node

Reads the gamepad and publishes it on `joy`. It is [`resilient_joy_linux.cpp`](./src/resilient_joy_linux.cpp), a copy of `joy_linux_node` from [joystick_drivers](https://github.com/ros-drivers/joystick_drivers) 3.3.0 with the same topics and parameters. The header of the file lists the changes. In short:

- it exits cleanly on SIGINT and SIGTERM, within a second; a second signal is ignored, and launch escalates to SIGKILL if it hangs;
- while no gamepad is connected it looks up the device number of `dev` in `/sys/dev/char` before opening it, so a device node without a gamepad behind it does not make the kernel request the char-major-13 module (a modprobe call) every second. Symlinks and nodes remapped into a container are checked by the device they point at;
- a gamepad is optional: the `joy_node: Joystick Driver Status` diagnostic is OK when no gamepad was ever connected, OK while one is connected, and WARN when a connected gamepad is lost. The stock node reports ERROR whenever no gamepad is open. The "Couldn't open joystick" line follows the same rule: INFO without a gamepad, WARN after one is lost, where the stock node logs ERROR.
