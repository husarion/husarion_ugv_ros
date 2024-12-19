# husarion_ugv_description

The package contains URDF files responsible for creating a representation of the robot by specifying the relationships and types of connections (joints) between individual links. It also contains information about the robot's mesh.

## Launch Files

- `load_urdf.launch.py` - loads the robot's URDF and creates simple bindings to display moving joints.
- `overwrite_robot_description.launch.py` - launch is able to change `robot_description` topic in runtime.
- `rviz.launch.py` - launch RViz2 with basic configuration.

## Configuration Files

- [`components.yaml`](./config/components.yaml): Allows you to quickly add visualization of sensors, TF connections and simulate their behavior in the simulator.
- [`WH01.yaml`](./config/WH01.yaml): Description of physical and visual parameters for the wheel WH01.
- [`WH02.yaml`](./config/WH02.yaml): Description of physical and visual parameters for the wheel WH02.
- [`WH04.yaml`](./config/WH04.yaml): Description of physical and visual parameters for the wheel WH04.
- [`WH05.yaml`](./config/WH05.yaml): Description of physical and visual parameters for the wheel WH05.
