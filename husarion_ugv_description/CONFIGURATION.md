# husarion_ugv_description

## URDF - Robot Model Configuration

### Configure Components

Create `components.yaml` file with the desired components. Here's a sample configuration:

```yaml
components:
  - type: MAN02
    parent_link: cover_link
    xyz: 0.2 -0.2 0.0
    rpy: 0.0 0.0 0.0
    device_namespace: ur5
```

In this example:
    - `type`: MAN02: Specifies the component type.
    - `parent_link`: cover_link: Defines the parent link to attach the component.
    - `xyz`: 0.2 -0.2 0.0: Sets the position of the component.
    - `rpy`: 0.0 0.0 0.0: Sets the rotation of the component.
    - `device_namespace`: ur5: Sets the device namespace.

### Visualize Robot Model Configuration

To use the `ros2 launch` command to launch the `visualize_fake_robot.launch.py` file with a specified robot model and components configuration path, follow these steps:

1. Open a terminal.
2. Source build workspace.
3. Execute the following command:

```bash
ros2 launch husarion_ugv_description visualize_fake_robot.launch.py robot_model:=lynx use_joint_state_publisher:=true components_config_path:=$(pwd)/components.yaml
```

If you want to move the wheels or move the manipulator you can run the launch file with `joint_state_publisher_gui`:

```bash
ros2 launch husarion_ugv_description visualize_fake_robot.launch.py robot_model:=lynx use_joint_state_publisher_gui:=true components_config_path:=$(pwd)/components.yaml
```

### Overwrite Robot Model Configuration

To change the configuration let's add another manipulator to `components.yaml`:

```yaml
components:
  - type: MAN02
    parent_link: cover_link
    xyz: 0.2 -0.2 0.0
    rpy: 0.0 0.0 0.0
    device_namespace: left_ur5
  - type: MAN02
    parent_link: cover_link
    xyz: 0.2 0.2 0.0
    rpy: 0.0 0.0 0.0
    device_namespace: right_ur5
```

To use the `ros2 launch` command to launch the `overwrite_robot_description.launch.py` file with the appropriate arguments to overwrite the robot model, follow these steps:

1. Open a terminal.
2. Source workspace.
3. Execute the following command:

```bash
ros2 launch husarion_ugv_description overwrite_robot_description.launch.py robot_model:=lynx components_config_path:=$(pwd)/components.yaml
```
