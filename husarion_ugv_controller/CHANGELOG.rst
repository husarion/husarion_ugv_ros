^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package husarion_ugv_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.0 (2025-05-19)
------------------
* Revert "Update changelog"
* Revert "2.3.0"
* Revert "2.3.0"
* Revert "Update changelog"
* Reapply "Release 2.3.0 to ros2"
* Merge branch 'ros2' of https://github.com/husarion/husarion_ugv_ros into rel-test
* Merge pull request `#550 <https://github.com/husarion/husarion_ugv_ros/issues/550>`_ from husarion/release-2.3.0
* 2.3.0
* Update changelog
* Merge pull request `#549 <https://github.com/husarion/husarion_ugv_ros/issues/549>`_ from husarion/revert-546-release-2.3.0
* Revert "Release 2.3.0"
* Merge pull request `#548 <https://github.com/husarion/husarion_ugv_ros/issues/548>`_ from husarion/revert-547-2.3.0-20250425
* Revert "Release 2.3.0 to ros2"
* Merge pull request `#547 <https://github.com/husarion/husarion_ugv_ros/issues/547>`_ from husarion/2.3.0-20250425
* Merge pull request `#546 <https://github.com/husarion/husarion_ugv_ros/issues/546>`_ from husarion/release-2.3.0
* 2.3.0
* Update changelog
* update wheel parameters (`#544 <https://github.com/husarion/husarion_ugv_ros/issues/544>`_)
* Merge remote-tracking branch 'origin/ros2-devel' into change-pat
* Jazzy load urdf (`#520 <https://github.com/husarion/husarion_ugv_ros/issues/520>`_)
* Merge pull request `#518 <https://github.com/husarion/husarion_ugv_ros/issues/518>`_ from husarion/jazzy-devel-hw
* Merge branch 'ros2-devel' into jazzy-devel-hw
* Migrate simulation code to run on ROS2 Jazzy (`#511 <https://github.com/husarion/husarion_ugv_ros/issues/511>`_)
* Update minimal cmake version
* Use `hardware` instead of private topic
* Merge branch 'jazzy-devel-sim' into jazzy-devel-hw
* Merge branch 'ros2-devel' into jazzy-devel-sim
* Fix controllers
* Merge branch 'jazzy-devel-sim' into jazzy-devel-hw
* Use stamped msgs
* Merge branch 'ros2-devel' into jazzy-devel-sim
* Contributors: Dawid Kmak, Rafal Gorecki, action-bot, github-actions[bot], kmakd, rafal-gorecki, rafal.gorecki

2.2.1 (2025-04-04)
------------------
* Merge pull request `#509 <https://github.com/husarion/husarion_ugv_ros/issues/509>`_ from husarion/ros2-imu-ros2-controll-fix
* Workaround and remove ros2_controllers custom fork
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-shutdown-request
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-enchance-performance
* Contributors: Dawid Kmak, kmakd, rafal-gorecki

2.2.0 (2025-03-13)
------------------
* release fixes (`#497 <https://github.com/husarion/husarion_ugv_ros/issues/497>`_)
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-devel
* Merge pull request `#496 <https://github.com/husarion/husarion_ugv_ros/issues/496>`_ from husarion/repo-rename
* update links
* Merge branch 'ros2-devel' into lights-new
* Add log level argument to launch files (`#473 <https://github.com/husarion/husarion_ugv_ros/issues/473>`_)
* adjust wheel_separation_multiplier (`#485 <https://github.com/husarion/husarion_ugv_ros/issues/485>`_)
* Merge pull request `#479 <https://github.com/husarion/husarion_ugv_ros/issues/479>`_ from husarion/e_stop_torque_enable
* Rename to motor_torque_enable
* Rename service e_stop_torque_enable
* Add suggestions
* Change logic from `motor_power_enable` to `e_stop_torque_enable`
* Fix and update config script (rename panther_description -> husarion_ugv_description)
* Merge lynx_description and panther_description into husarion_ugv_descriptions (`#456 <https://github.com/husarion/husarion_ugv_ros/issues/456>`_)
* Readme files minor fixes  (`#463 <https://github.com/husarion/husarion_ugv_ros/issues/463>`_)
* Merge pull request `#466 <https://github.com/husarion/husarion_ugv_ros/issues/466>`_ from husarion/ros2-add-msgs
* Merge branch 'ros2-devel' into ros2-add-msgs
* Merge branch 'ros2-devel' into add-panther-diagnostics-config
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-devel
* Merge pull request `#460 <https://github.com/husarion/husarion_ugv_ros/issues/460>`_ from husarion/ros2-devel-better-config-dir
* Add husarion_ugv_msgs
* optional config dir
* Merge pull request `#457 <https://github.com/husarion/husarion_ugv_ros/issues/457>`_ from husarion/ros2-lynx-devel
* decrease cmd_vel_timeout
* Merge pull request `#455 <https://github.com/husarion/husarion_ugv_ros/issues/455>`_ from husarion/ros2-lynx-merge
* use ROBOT_MODEL_NAME env
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-lynx-devel
* Merge pull request `#444 <https://github.com/husarion/husarion_ugv_ros/issues/444>`_ from husarion/ros2-config-dir
* component file
* use configs from /config dir
* Ros2 husarion ugv v2 (`#422 <https://github.com/husarion/husarion_ugv_ros/issues/422>`_)
* Contributors: BOOTCFG, Dawid Kmak, Jakub Delicat, Miłosz Łagan, Rafal Gorecki, Stefan, kmakd, rafal-gorecki

2.1.2 (2024-12-02)
------------------
* Merge branch 'ros2-devel' into ros2-lights-tests
* Contributors: pawelirh

2.1.1 (2024-09-05)
------------------
* Merge pull request `#403 <https://github.com/husarion/panther_ros/issues/403>`_ from husarion/ros2-control-ns-fix
* Remove deprecated --namespace arg
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-testing-poc
* Ros2 estop sim gui (`#384 <https://github.com/husarion/panther_ros/issues/384>`_)
* Merge branch 'ros2-devel' into ros2-ns-refactor
* unify CMakeLists.txt files (`#381 <https://github.com/husarion/panther_ros/issues/381>`_)
* unify CMakeLists.txt files
* New format of documentation  (`#369 <https://github.com/husarion/panther_ros/issues/369>`_)
* Contributors: Dawid, Dawid Kmak, pawelirh, rafal-gorecki

2.1.0 (2024-08-02)
------------------
* Fix imu tf frame (`#373 <https://github.com/husarion/panther_ros/issues/373>`_)
* Process noise update (`#361 <https://github.com/husarion/panther_ros/issues/361>`_)
* Merge pull request `#362 <https://github.com/husarion/panther_ros/issues/362>`_ from husarion/ros2-api-reorganization
* Enhance ROS API names in the stack
* Contributors: Dawid Kmak, pawelirh, rafal-gorecki

2.0.4 (2024-06-28)
------------------
* Add EKF GPS configuration (`#351 <https://github.com/husarion/panther_ros/issues/351>`_)
* Merge pull request `#343 <https://github.com/husarion/panther_ros/issues/343>`_ from husarion/ros2-gpio-controller-revision
* Merge branch 'ros2-devel' into ros2-gz-lights
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-gpio-controller-revision
* Merge branch 'ros2' into ros2-build-in-animation
* Implement changes in driver node
* Merge branch 'ros2' into ros2-gz-lights
* Contributors: Dawid Kmak, pawelirh, rafal-gorecki

2.0.3 (2024-06-06)
------------------
* Merge pull request `#320 <https://github.com/husarion/panther_ros/issues/320>`_ from husarion/ros2-clear-logs
* Organize logs: panther_system and panther_imu
* Contributors: Dawid Kmak, pawelirh

2.0.2 (2024-06-05)
------------------
* Launch refactor (`#307 <https://github.com/husarion/panther_ros/issues/307>`_)
* Merge branch 'ros2' of https://github.com/husarion/panther_ros into ros2-manager-refactor
* added diagnostics remapping and namespace to system_status (`#306 <https://github.com/husarion/panther_ros/issues/306>`_)
* added remappings to diagnostics
* Ros2 add components (`#277 <https://github.com/husarion/panther_ros/issues/277>`_)
* Merge pull request `#304 <https://github.com/husarion/panther_ros/issues/304>`_ from husarion/ros2-control-fix
* Merge branch 'ros2' into ros2-control-fix
* fix ekf
* Merge pull request `#303 <https://github.com/husarion/panther_ros/issues/303>`_ from husarion/ros2-controler-patch
* Patch
* Fix controller with namespace
* Merge branch 'ros2' of https://github.com/husarion/panther_ros into ros2-manager-refactor
* Multi robot spawn working (`#256 <https://github.com/husarion/panther_ros/issues/256>`_)
* Contributors: Dawid, Jakub Delicat, Paweł Irzyk, rafal-gorecki

2.0.1 (2024-05-01)
------------------
* Merge pull request `#261 <https://github.com/husarion/panther_ros/issues/261>`_ from husarion/ros2-readme
* Pawel sugestions
* Merge branch 'ros2-devel' into ros2-readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Ros2 control imu hardware (`#236 <https://github.com/husarion/panther_ros/issues/236>`_)
* Add controller readme
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-os-diagnostics
* Contributors: Jakub Delicat, Paweł Irzyk, rafal-gorecki

2.0.0 (2024-03-29)
------------------
* Ros2 namespace (`#255 <https://github.com/husarion/panther_ros/issues/255>`_)
  * Preparation for namespace
  * Simulation working
  * Hardware look ok
  * Update panther_controller/config/WH01_controller.yaml
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  * Apply Jakub suggestions
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  * Fix imu
  * Readme
  * Add imu namespace
  * Jakub suggestions
  * Add panther manager to xml
  * pre-commit
  * Fixed ekf
  * Additional remapping
  * fix imu
  * Pawel suggestions (collision with gamepad)
  * cmd_vel
  * Use namespace instead of PushRosNamespace
  ---------
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  Co-authored-by: Jakub Delicat <jakub.delicat@husarion.com>
* Merge pull request `#257 <https://github.com/husarion/panther_ros/issues/257>`_ from husarion/ros2-headers
  Divide Headers into std and local liblaries
* Headers + Copyright
* Merge pull request `#246 <https://github.com/husarion/panther_ros/issues/246>`_ from husarion/ros2-panther-manager
  ROS 2 panther_manager
* Add launch behavior
* Merge branch 'ros2-devel' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Merge pull request `#240 <https://github.com/husarion/panther_ros/issues/240>`_ from husarion/ekf_optimalization
  Ekf optimalization
* Merge branch 'ros2-devel' into ros2-ekf-optimalization
* Merge branch 'ros2-devel' into ros2-lights-tests
* Merge branch 'ros2-manager-plugins' of https://github.com/husarion/panther_ros into ros2-panther-manager
* Update panther_controller/config/WH04_controller.yaml
* Update panther_controller/config/WH04_controller.yaml
* Add comments
* update covariances
* Merge pull request `#251 <https://github.com/husarion/panther_ros/issues/251>`_ from husarion/ros2-build-depend
  Hardware / Sim Dependencies
* Add Readme
* Add Readme
* HW/SIM Dependencies
* Changed controllers spawn timeout to 10 (`#248 <https://github.com/husarion/panther_ros/issues/248>`_)
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-manager-plugins
* Update covariance
* Merge pull request `#235 <https://github.com/husarion/panther_ros/issues/235>`_ from husarion/ros2-dependencies
  Fix dependencies
* Update panther_controller/package.xml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Fix dependency
* Merge pull request `#227 <https://github.com/husarion/panther_ros/issues/227>`_ from husarion/ros2-add-mecanum-controller
  Add mecanum controller
* update
* Merge remote-tracking branch 'origin/ros2-devel' into ros2-add-mecanum-controller
* Merge pull request `#208 <https://github.com/husarion/panther_ros/issues/208>`_ from husarion/ros2-control
  Add ROS 2 control
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_gpiod/CMakeLists.txt
  panther_gpiod/package.xml
  panther_gpiod/src/gpio_driver.cpp
* Add GPIO controller (`#222 <https://github.com/husarion/panther_ros/issues/222>`_)
  * add GPIO controller
  * Basic integration of gpio controller and panther system
  * [WIP] Add panther version
  * add io state topic
  * Remove unnecessary parts from cmakelists
  * Cleanup gpio controller
  * Add estop to panther system
  * Add todo comment
  * Add ServiceWrapper
  * Add estop services
  * Add remaps to ros2 control
  * Add publishing estop state, change iostate to latched and fix publishing initial state
  * revise e-stop logic in initial stage
  * same, but in better way
  * small changes
  * remove clear_errors service
  * Fix test
  * Add resetting gpio controller
  * Change wheel separation multiplier to 1.0
  * fix pin names list
  * add robot version check before GPIO read
  * Change lock in gpio driver
  * Fix order in cmakelists
  * Change throws to exception in briefs
  * Remove unnecessary includes
  * Fix controller_manager topic remaps
  * Add checking if last commands were 0 before resetting estop
  * Change estop variable to atomic bool
  * Add motor controller mutex
  * Change order of operations when setting estop
  * Fix order of methods
  * Fixes in panther system - change methods order, use ReadDriverStatesUpdateFrequency, remove unnecessary logs
  * Remove max_safety_stop_attempts (no longer needed after adding gpio controller)
  * Refactor setting estop in write method
  * Fix estop naming convention
  * Remove old todos
  * Fix typo
  * Review fixes
  * fix formatting
  * Update panther_hardware_interfaces/include/panther_hardware_interfaces/gpio_controller.hpp
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
  * review fixes
  * rename some methods
  * draft of InitializeAndPublishIOStateMsg functionality
  * fix io_state topic
  * fix service warappers
  * small fix
  * rewiew fixes
  * add briefs in gpio_controler
  * review fixes
  * small fix
  ---------
  Co-authored-by: Paweł Kowalski <kowalski.pawel.r@gmail.com>
  Co-authored-by: Paweł Kowalski <82044322+pkowalsk1@users.noreply.github.com>
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Add IMU noise + basic EKF configuration (`#229 <https://github.com/husarion/panther_ros/issues/229>`_)
  * Fix collisions
  * remove parent dir
  * Add IMU noise
  * EKF working
  * Add controller
  * Update panther_bringup/config/ekf.yaml
  * Update panther_bringup/config/ekf.yaml
  * Format
* update
* update
* update
* clean
* Add mecanum controller
* Merge pull request `#219 <https://github.com/husarion/panther_ros/issues/219>`_ from husarion/ros2-control-pdo-commands
  ros2_control PDO commands
* Merge branch 'ros2-control' into ros2-control-pdo-commands
  Conflicts:
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
  panther_description/urdf/panther_macro.urdf.xacro
  panther_hardware_interfaces/CMakeLists.txt
  panther_hardware_interfaces/CODE_STRUCTURE.md
  panther_hardware_interfaces/README.md
  panther_hardware_interfaces/include/panther_hardware_interfaces/canopen_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/motors_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system_ros_interface.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_data_converters.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_driver.hpp
  panther_hardware_interfaces/src/canopen_controller.cpp
  panther_hardware_interfaces/src/motors_controller.cpp
  panther_hardware_interfaces/src/panther_system.cpp
  panther_hardware_interfaces/src/panther_system_ros_interface.cpp
  panther_hardware_interfaces/src/roboteq_driver.cpp
* Merge branch 'ros2-devel' into ros2-control-pdo-commands
  Conflicts:
  panther_bringup/launch/bringup.launch.py
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_bringup/launch/bringup.launch.py
  panther_controller/config/WH01_controller.yaml
  panther_controller/config/WH02_controller.yaml
  panther_controller/config/WH04_controller.yaml
* Manuall merge of ros2-prealpha to ros2-dev (`#218 <https://github.com/husarion/panther_ros/issues/218>`_)
  * manually merge prealpha with ros2-dev
  * typo and formatting
  * change locks and simplify code
  * add missing library
  * fix build
* CR suggestions
* CR suggestions - remove effort controller
* Remove todos
* Update controllers config
* Change to 100Hz and increase allowed number of errors
* Change frequency to 125hz
* Merge branch 'ros2-control' into ros2-control-pdo-commands
  Conflicts:
  panther_hardware_interfaces/README.md
  panther_hardware_interfaces/include/panther_hardware_interfaces/canopen_controller.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/panther_system.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_data_converters.hpp
  panther_hardware_interfaces/include/panther_hardware_interfaces/roboteq_driver.hpp
  panther_hardware_interfaces/src/motors_controller.cpp
  panther_hardware_interfaces/src/panther_system.cpp
  panther_hardware_interfaces/src/roboteq_driver.cpp
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  panther_bringup/launch/bringup.launch.py
* Fix launches
* fix controller launch
* Add odom topic remap
* Update todo comments
* Change controller frequency to 50hz
* Fix update rates in controllers
* Merge pull request `#212 <https://github.com/husarion/panther_ros/issues/212>`_ from husarion/ros2-imu-node
  ROS 2 imu node
* Update controller configs
* add use_sim condition
* read imu position from env
* Precommit changes
* Merge branch 'ros2-devel' into ros2-control
  Conflicts:
  .clang-format
  README.md
  panther_controller/CMakeLists.txt
  panther_controller/launch/controller.launch.py
  panther_controller/package.xml
  panther_description/CMakeLists.txt
  panther_description/config/WH01.yaml
  panther_description/config/WH02.yaml
  panther_description/config/WH04.yaml
  panther_description/meshes/WH01/fl_wheel.dae
  panther_description/meshes/WH01/fr_wheel.dae
  panther_description/meshes/WH01/rl_wheel.dae
  panther_description/meshes/WH01/rr_wheel.dae
  panther_description/meshes/WH02/fl_wheel.dae
  panther_description/meshes/WH02/fr_wheel.dae
  panther_description/meshes/WH02/rl_wheel.dae
  panther_description/meshes/WH02/rr_wheel.dae
  panther_description/meshes/WH04/fl_wheel.dae
  panther_description/meshes/WH04/fr_wheel.dae
  panther_description/meshes/WH04/rl_wheel.dae
  panther_description/meshes/WH04/rr_wheel.dae
  panther_description/meshes/body.dae
  panther_description/meshes/components/external_antenna.dae
  panther_description/package.xml
  panther_description/rviz/panther.rviz
  panther_description/urdf/body.urdf.xacro
  panther_description/urdf/components/external_antenna.urdf.xacro
  panther_description/urdf/panther.urdf.xacro
  panther_description/urdf/panther_macro.urdf.xacro
  panther_description/urdf/wheel.urdf.xacro
* Add comment to controller
* Add pre-commit, clang-format and license to files (`#207 <https://github.com/husarion/panther_ros/issues/207>`_)
  Add pre-commit, clang-format and license to files
* Merge pull request `#201 <https://github.com/husarion/panther_ros/issues/201>`_ from husarion/ros2-gazebo
  Ros2 gazebo
* review fixes
* add puslish_robot_state param in all files
* add new launch params
* review fixes
* Update panther_controller/package.xml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/launch/controller.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/launch/controller.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/launch/controller.launch.py
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/config/WH04_controller.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/config/WH02_controller.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/config/WH02_controller.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/config/WH02_controller.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* Update panther_controller/config/WH02_controller.yaml
  Co-authored-by: Dawid Kmak <73443304+KmakD@users.noreply.github.com>
* fix controller and ekf covariance
* fix deps
* grammar fixes
* add battery plugin
* add wheel params in launches
* add imu filter and ekf
* initial sim configuration draft
* Add ros2 control
* Contributors: Dawid, Dawid Kmak, Jakub Delicat, Krzysztof Wojciechowski, Maciej Stępień, Paweł Irzyk, Paweł Kowalski, rafal-gorecki
