# Animation API

## Animation Class

Basic animation type. This class consists of:

Arguments:

- `animation_description` [*dict*]: a dictionary with animation description. Contain following keys:
  - `brightness` [*float*, optional]: will be assigned to the `brightness_` variable as a value in a range **[0, 255]**.
  - `duration` [*float*]: will be assigned to `duration_` variable.
  - `repeat` [*int*, optional]: will be assigned to `loops_` variable.
- `num_led` [*int*]: number of LEDs in a bumper.
- `controller_frequency` [*float*]: controller frequency **[Hz]** at which animation frames will be processed.

Functions:

- `Initialize` - this method handles animation initialization. It should be invoked right after the animation is created, as the constructors for pluginlib classes cannot have parameters.
- `Update` - returns a list of length `num_led` with **RGBA** values of colors to be displayed on the Bumper Lights based on the `UpdateFrame` method. Handles looping over animation based on `repeating` parameter, iterating over animation, and updating its progress.
- `UpdateFrame` - returns a list of shape (`num_led`, 4) with **RGBA** values of colors to be displayed on the Bumper Lights. Colors are described as a list of integers with respective **R**, **G**, and **B** color values and **A** alpha channel. By default, not implemented.
- `Reset` - resets animation to its initial state. If overwritten, it requires calling the parent class implementation first.

Setters:

- `SetParam` - allows processing an additional parameter passed to the animation when it is created.

Getters:

- `IsFinished` [*bool*]: returns a value indicating if the animation execution process has finished.
- `GetNumberOfLeds` [*std::size_t*]: returns the number of LEDs.
- `GetProgress` [*float*]: returns animation execution progress.
- `GetAnimationLength` [*std::size_t*]: returns animation length.
- `GetAnimationIteration` [*std::size_t*]: returns current animation iteration.

## Defining a New Animation Type

It is possible to define your own animation type with expected, new behavior. All animation definitions inherit from the basic `Animation` class. Animations are loaded using `pluginlib` and can be defined from any point in your project. All you need is to import `Animation` class from `husarion_ugv_lights` package and export it as a pluginlib plugin. For more information about creating and managing pluginlib see: [Creating and using plugins (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

Frames are processed in ticks with a frequency of `controller_frequency`. It is required to overwrite the `UpdateFrame` method which must return a list representing the frame for a given tick. The advised way is to use the `GetAnimationIteration` (current animation tick) as a time base for animation frames. The length of the frame has to match `num_led`. Each element of the frame represents a color for a single LED in the Bumper Lights. Colors are described as a list of integers with respective **R**, **G**, and **B** color values and **A** alpha channel. Additional parameters (e.g. image path) can be passed within `animation_description` and processed inside `Initialize` method. See the example below or other animation definitions.

Create a New Animation Type:

```c++
# my_cool_animation.hpp

#ifndef MY_PACKAGE_MY_COOL_ANIMATION_HPP_
#define MY_PACKAGE_MY_COOL_ANIMATION_HPP_

#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/animation/animation.hpp"

class MyCoolAnimation : public Animation
{
public:
  MyCoolAnimation() {}
  ~MyCoolAnimation() {}

  void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency) override;

private:
  std::uint8_t value_;
};

#endif  // MY_PACKAGE_MY_COOL_ANIMATION_HPP_
```

```c++
# my_cool_animation.cpp

#include "yaml-cpp/yaml.h"

#include "my_package/animation/my_cool_animation.hpp"

void MyCoolAnimation::Initialize(
  const YAML::Node & animation_description, const std::size_t num_led,
  const float controller_frequency)
{
  Animation::Initialize(animation_description, num_led, controller_frequency);
}

void MyCoolAnimation::SetParam(const std::string & param)
{
  value_ = static_cast<std::uint8_t>(param);
}

std::vector<uint8_t> MyCoolAnimation::UpdateFrame()
{
  return std::vector<std::uint8_t>(this->GetNumberOfLeds() * 4, value_);
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(my_package::MyCoolAnimation, my_package::Animation)

```
