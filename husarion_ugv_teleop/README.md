# husarion_ugv_teleop

The package contains configuration and launch files necessary to control the robot via gamepad.

## Launch Files

- `teleop.launch.py`: Enables robot teleoperation via gamepad by loading the gamepad controller `joy2twist`.

## Configuration Files

- `joy2twist_${ROBOT_MODEL_NAME}.yaml`: Describes button mappings of a gamepad and parameters such as linear and angular velocity.
