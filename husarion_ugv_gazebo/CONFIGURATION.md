# husarion_ugv_gazebo

## Use of GPS in Simulation

The NavSat system is utilized to publish the robot's position within the Gazebo world. It manages navigation satellite sensors, such as GPS, which report position and velocity in spherical coordinates (latitude/longitude) through Ignition Transport.

The NavSat sensors requires the spherical coordinates of the world origin to be configured. This configuration can be accomplished, for instance, by employing the `<spherical_coordinates>` tag within the world's SDF or by utilizing the Ignition `/world/world_name/set_spherical_coordinates` service.

To obtain GPS data in Ignition, follow these steps:

- Include the [ANT02](https://github.com/husarion/ros_components_description/blob/ros2/urdf/teltonika_003R-00253.urdf.xacro) by adding the following lines to your [components.yaml](../husarion_ugv_description/config/components.yaml) file inside the `components` list:

```yaml
  - type: ANT02
    parent_link: cover_link
    xyz: 0.185 -0.12 0.0
    rpy: 0.0 0.0 3.14
    device_namespace: gps
```

- Add the following tag to your world's SDF file and specify this file using the `world` parameter (the default `husarion_world.sdf` file already includes this tag):

```xml
<spherical_coordinates>
  <surface_model>EARTH_WGS84</surface_model>
  <world_frame_orientation>ENU</world_frame_orientation>
  <latitude_deg>53.1978</latitude_deg>
  <longitude_deg>18.3732</longitude_deg>
  <elevation>0</elevation>
  <heading_deg>0</heading_deg>
</spherical_coordinates>
```

## Linear Battery Plugin

It is possible to simulate the battery operation of the simulated robot. By default, this feature is disabled, but you can enable it by setting the `simulate_discharging` parameter to `true` in the `battery_plugin.yaml` file or in the file pointed to by the `battery_config_path` parameter. Below, you will find the plugin parameters that enable battery simulation.

- `simulate_discharging` [*bool*, default: **false**]: Enables battery simulation. If set to `true`, the battery will discharge **at a constant rate** (regardless of joint torque), and if it depletes completely, the robot will stop moving. When set to `false`, the battery will not discharge, but the battery status information will still be published on the `battery/battery_status` topic.
- `initial_charge_percentage` [*float*, default: **70.0**]: Sets the initial charge percentage of the battery.
- `charging_time` [*float*, default: **6.0**]: Specifies the charging time for the battery in hours.
- `power_load` [*float*, default: **120.0**]: Represents the average power load during normal operation **[W]** and is initially set to 120.0 W. With the default power load of 120.0 W, the robot can operate for up to 8 hours. When the `simulate_discharging` parameter is set to `false`, this value defaults to 0.0 W. Users are encouraged to customize this value to match their specific requirements. For more information on robot power consumption, please refer to [Battery & Charging Documentation](https://husarion.com/manuals/panther/#battery--charging).

> [!NOTE]
>
> The battery model is quite simple and involves significant simplifications. As a result, the battery discharge rate observed on the physical robot may differ from the model's predictions.

### Charging Process

Unfortunately, there is no straightforward way to exchange `LinearBatteryPlugin` services between ROS and Gazebo Transport, so you need to use the `ign` commands. As a result, the method of charging differs between the real and simulated robot.

You can start the charging process by calling the Ignition service:

```bash
ign service --service /model/{robot_model}/battery/battery/recharge/start --reqtype ignition.msgs.Boolean --reptype ignition.msgs.Empty --req '' --timeout 0
```

and stop it using:

```bash
ign service --service /model/{robot_model}/battery/battery/recharge/stop --reqtype ignition.msgs.Boolean --reptype ignition.msgs.Empty --req '' --timeout 0
```
