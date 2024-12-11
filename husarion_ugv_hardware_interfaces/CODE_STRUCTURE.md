# Structure

A brief introduction to the code structure of the Husarion UGV system.

## DriverInterface

Interface to manage robot driver.
Implementations:

* `RoboteqDriver`: Low-level CANopen driver implementing LoopDriver from [Lely](https://opensource.lely.com/canopen/).
It takes care of translating CANopen indexes into meaningful data.
Provided methods can be used for sending commands and reading all the useful parameters from the Roboteq drivers (they abstract low level SDO and PDO communication).
Timestamp of all received PDO data is also saved, which can be later used for detecting timeout errors. Also, manages `MotorDrivers`.

## MotorDriverInterface

Abstract interface for managing each motor connected to the driver.
Implementations:

* `RoboteqMotorDriver`: Responsible for reading state and sending command velocities with usage of `RoboteqDriver` interfaces.

## CANopenManager

Takes care of CANopen communication - creates and initializes master controller. For handling CANopen communication separate thread is created with configurable RT priority.

## RobotDriver

Interface to control robot drivers.
Implementations:

* `RoboteqRobotDriver`: This class abstracts the usage of Roboteq controllers. It uses canopen_controller for communication with Roboteq controllers, implements the activation procedure for controllers (resets script and sends initial 0 command), and provides methods to get data feedback and send commands. Data is converted between raw Roboteq formats and SI units using `roboteq_data_converters`.
It has two concrete implementations:
  * `LynxRobotDriver`: Contains one Roboteq controller.
  * `PantherRobotDriver`: Contains two Roboteq controllers.

## RoboteqDataConverters

Provides a few classes for converting data in raw Roboteq formats read from Roboteq drivers into appropriate units or message formats. It can be divided into two types, command and data feedback. The command provides one utility function that converts a command in rad/s into a Roboteq command and returns it:

* `RoboteqVelocityCommandConverter`

Data feedback converters also store data (it is passed using Set methods, and later converted data can be read using Get data).

* `MotorState` - converts position, velocity and torque feedback
* `FaultFlag`, `ScriptFlag`, `RuntimeError` - converts flag error data into messages
* `RoboteqDriverState` - temperature, voltage, and current

Feedback converters are combined in the `DriverData` class to provide the full state of one controller. It consists of

* 2 `MotorState` (left and right)
* `FaultFlag`, `ScriptFlag`
* 2 `RuntimeError` (for left and right motors)
* `RoboteqDriverState`

## RoboteqErrorFilter

A class that keeps track of different types of errors. In some rare cases, Roboteq controllers can miss for example the SDO response, or PDO can be received a bit later, which results in a timeout.
As they usually are rare and singular occurrences, it is better to filter some of these errors and escalate only when a certain number of errors happen.

## GPIODriver

The GPIODriver is a low-level class responsible for direct interaction with the GPIO (General Purpose Input/Output) pins on the Raspberry Pi.
It comprises a wrapper implementation for the GPIOD library, enabling real-time manipulation of GPIO pins on the Raspberry Pi. Offering convenient interfaces for setting pin values, altering their direction, monitoring events, and conducting other GPIO operations, this library facilitates effective GPIO pin management on the Husarion UGV. It simplifies integration within robotic applications.

## GPIOController

The GPIOController provides wrappers for the GPIO driver, handling reading and writing pins of the RPi GPIO. It includes the following utilities:

* `GPIOControllerInterface`: Interface for all wrappers that handle GPIO control tasks.
* `GPIOController`: Class with specific logic for Lynx and Panther robot.
* `Watchdog`: Entity responsible for spinning the software Watchdog. It periodically sets the high and low states of specific GPIO Watchdog pin. Used only with `GPIOController`.

## EStop

Implementation of emergency stop handling.

* `EStopInterface`: Interface for versioned emergency stop implementations.
* `EStop`: Class with specific logic for the Husarion UGV.

## SystemRosInterface

A class that takes care of additional ROS interface of Husarion UGV system, such as publishing driver state and providing service for clearing errors.

## {Robot}System

The main class that implements SystemInterface from ros2_control (for details refer to the [ros2_control documentation](https://control.ros.org/master/index.html)).
Handles transitions (initialization, activation, shutdown, error, etc.), provides interfaces for feedback (position, velocity, effort) and commands (velocity).
In the main loop controller should call `read` and `write` functions to communicate with motor drivers.
