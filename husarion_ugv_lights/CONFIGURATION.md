# husarion_ugv_lights

## LED Animations

Basic led configuration is loaded from [`{robot_model}_animations.yaml`](config) file. It includes definition of robot panels, virtual segments and default animations. The default appearance of the animation when looking at the robot from the front or back is as follows:

|  ID   | NAME              | PRIORITY | ANIMATION                                          |
| :---: | ----------------- | :------: | -------------------------------------------------- |
|   0   | E_STOP            |    3     | ![E_STOP](.docs/E_STOP.webp)                       |
|   1   | READY             |    3     | ![READY](.docs/READY.webp)                         |
|   2   | ERROR             |    1     | ![ERROR](.docs/ERROR.webp)                         |
|   3   | MANUAL_ACTION     |    3     | ![MANUAL_ACTION](.docs/MANUAL_ACTION.webp)         |
|   4   | AUTONOMOUS_ACTION |    3     | ![AUTONOMOUS_ACTION](.docs/AUTONOMOUS_ACTION.webp) |
|   5   | GOAL_ACHIEVED     |    2     | ![GOAL_ACHIEVED](.docs/GOAL_ACHIEVED.webp)         |
|   6   | LOW_BATTERY       |    2     | ![LOW_BATTERY](.docs/LOW_BATTERY.webp)             |
|   7   | CRITICAL_BATTERY  |    2     | ![CRITICAL_BATTERY](.docs/CRITICAL_BATTERY.webp)   |
|   9   | CHARGING_BATTERY  |    3     | ![CHARGING_BATTERY](.docs/CHARGING_BATTERY.webp)   |

### Panels

The `panels` section of the YAML file lists all the physical LED panels on the robot. Each panel has two attributes:

- `channel` [*int*, default: **None**] the identifier for the LED panel. It is used to differentiate between multiple panels.
- `number_of_leds`: defines the total number of LEDs present on the panel.

### Segments

The `segments` section is used to create virtual segments on the robot by dividing the LED panels into different parts. This allows for more precise control over which LEDs are lit up for different effects or indicators. Each segment has three attributes:

- `name`: the identifier for the segment, such as "front" or "rear". It is used to differentiate between multiple segments.
- `channel`: This specifies which LED panel the segment belongs to. It has to match one of the channels defined in the `panels` section.
- `led_range`: This defines the range of LEDs within the panel that the segment covers. The range is specified as a start-end pair (e.g. 0-45). The range can be specified in reverse order (e.g. 45-0), which may be useful for wiring or orientation reasons.

### Segments map

The `segments_map` section allows creating named groups of segments on which animations can be displayed. Each entry under `segments_map` consists of a key representing the group name and a list of segments included in the group. Segment names have to match one of the segments defined in the `segments` section. By default, you can use provided mapping:

- `all` [*list*, default: **None**]: Grouping both `front` and `rear` segments together.
- `front` [*list*, default: **None**]: Containing only the `front` segment.
- `rear` [*list*, default: **None**]: Containing only the `rear` segment.

### Animations

The `led_animations` section contains a list with definitions for various animations that can be displayed on the LED segments. Supported keys are:

- `animations` [*list*, default: **None**]: definition of animation for each Bumper Lights. Supported keys are:
  - `type` [*string*, default **None**]: Specifies the type of animation. Default animation types are: `husarion_ugv_lights::ImageAnimation`, `husarion_ugv_lights::ChargingAnimation`.
  - `segments` [*string*, default **None**]: Indicates which segment mapping this particular animation applies to (e.g., all, front, rear).
  - `animation` [*yaml*, default: **None**]: An animation to be displayed on segments. The keys for the configuration of different animation types are explained in detail under the [**Animation Types**](#animation-types) section.
- `id` [*int*, default: **None**]: unique ID of an animation.
- `name` [*string*, default: **ANIMATION_`ID`**]: name of an animation. If not provided, it will default to **ANIMATION_`ID`**, where `ID` is equal to `id` parameter of the given animation.
- `priority` [*int*, default: **3**]: priority at which animation will be placed in the queue. The list below shows the behavior when an animation with a given ID arrives:
  - **1** interrupts and removes animation with priorities **2** and **3**.
  - **2** interrupts animations with priority **3**.
  - **3** adds animation to the end of the queue.
- `timeout` [*float*, default: **120.0**]: time in **[s]**, after which animation will be removed from the queue.

### Animation Types

#### Animation

Basic animation definition. Keys are inherited from the basic **Animation** class by all animations. Supported keys are:

- `brightness` [*float*, default: **1.0**]: animation brightness relative to APA102C driver `global_brightness`. The range between **[0.0, 1.0]**.
- `duration` [*float*, default: **None**]: duration of the animation.
- `repeat` [*int*, default: **1**]: number of times the animation will be repeated.

> [!NOTE]
> Overall display duration of an animation is a product of a single image duration and repeat count. The result of `duration` x `repeat`  can't exceed 10 **[s]**. If animation fails to fulfill the requirement, it will result in an error.

#### ImageAnimation

Animation of type `husarion_ugv_lights::ImageAnimation`, returns frames to display based on a supplied image. Extends `Animation` with keys:

- `color` [*int*, default: **None**]: The image is turned into grayscale, and then the color is applied with brightness from the gray image. Values have to be in HEX format. This parameter is not required.
- `image` [*string*, default: **None**]: path to an image file. Only global paths are valid. Allows using `$(find ros_package)` syntax.

#### ChargingAnimation

Animation of type `husarion_ugv_lights::ChargingAnimation`, returns frame to display based on `param` value representing Battery percentage. Displays a solid color with a duty cycle proportional to the Battery percentage. The color is changing from red (Battery discharged) to green (Battery fully charged).

### Defining Animations

Users can define their own LED animations using basic animation types. Similar to basic ones, user animations are parsed using YAML file and loaded on node start. For `ImageAnimation` you can use basic images from the `animations` folder and change their color with the `color` key ([see ImageAnimation](#imageanimation)). Follow the example below to add custom animations.

Create a YAML file with an animation description list. Example file:

```yaml
# my_awesome_user_animations.yaml
user_animations:
  # animation with default image and custom color
  - id: 21
    name: ANIMATION_1
    priority: 2
    animations:
      - type: husarion_ugv_lights::ImageAnimation
        segments: all
        animation:
          image: $(find husarion_ugv_lights)/animations/panther/strip01_red.png
          duration: 2
          repeat: 2
          color: 0xffff00

  # animation with custom image
  - id: 22
    name: ANIMATION_2
    priority: 3
    animations:
      - type: husarion_ugv_lights::ImageAnimation
        segments: all
        animation:
          image: /animations/custom_image.png
          duration: 3
          repeat: 1

  # animation with a custom image from custom ROS package
  - id: 23
    name: ANIMATION_3
    priority: 3
    animations:
      - type: husarion_ugv_lights::ImageAnimation
        segments: all
        animation:
          image: $(find custom_pkg)/animations/custom_image.png
          duration: 3
          repeat: 1

  # different animations for Front and Rear Bumper Light
  - id: 24
    name: ANIMATION_4
    priority: 3
    animations:
      - type: husarion_ugv_lights::ImageAnimation
        segments: front
        animation:
          image: $(find custom_pkg)/animations/front_custom_image.png
          duration: 2
          repeat: 2
      - type: husarion_ugv_lights::ImageAnimation
        segments: rear
        animation:
          image: $(find custom_pkg)/animations/rear_custom_image.png
          duration: 3
          repeat: 1
```

> [!IMPORTANT]
>
> - ID numbers from 0 to 19 are reserved for system animations.
> - Priority **1** is reserved for crucial system animations. Users can only define animations with priority **2** and **3**.

Remember to modify launch command to use user animations:

``` bash
ros2 launch husarion_ugv_bringup bringup.launch user_animations_file:=/my_awesome_user_animations.yaml
```

Test new animations:

```bash
ros2 service call /lights/set_animation husarion_ugv_msgs/srv/SetLEDAnimation "{animation: {id: 0, param: ''}, repeating: true}"
```

## Defining a Custom Animation Type

It is possible to define your own animation type with expected, new behavior. For more information, see: [**Animation API**](LIGHTS_API.md).
