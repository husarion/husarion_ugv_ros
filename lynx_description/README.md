# lynx_description

The package contains URDF files responsible for creating a representation of the robot by specifying the relationships and types of connections (joints) between individual links. It also contains information about the robot's mesh.

## Launch Files

- `load_urdf.launch.py` - loads the robot's URDF and creates simple bindings to display moving joints.

## Configuration Files

- [`components.yaml`](./config/components.yaml): Allows you to quickly add visualization of sensors, TF connections and simulate their behavior in the simulator.
- [`WH05.yaml`](./config/WH05.yaml): Description of physical and visual parameters for the wheel WH05.
