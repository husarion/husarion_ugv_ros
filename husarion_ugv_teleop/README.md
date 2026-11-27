# husarion_ugv_teleop

The package contains configuration and launch files necessary to control the robot via gamepad.

## Launch Files

- `teleop.launch.py`: Enables robot teleoperation via gamepad by loading the gamepad controller `joy2twist`.

### Launch Arguments

- `joy_dev` [*string*, default: **/dev/input/js0**]: Path to the joystick device file used by the `joy_linux` node. If the gamepad is not detected under the default path, check available devices with `ls /dev/input | grep js` and pass the correct one, e.g. `joy_dev:=/dev/input/js1`.

## Configuration Files

- `joy2twist_${ROBOT_MODEL_NAME}.yaml`: Describes button mappings of a gamepad and parameters such as linear and angular velocity.
