# Docker

Docker images dedicated to Husarion Panther ROS system and simulation.

## Docker Images

Docker images are automatically deployed to Docker Hub. The image tag includes information about the ROS distribution, the version of the [husarion_ugv_ros](https://github.com/husarion/husarion_ugv_ros/tree/ros2) repository, and the date of release. Additionally, stable image versions are tagged with `stable` and recommended for production use.
Below, you can find a list of available Docker images. To access the latest tag, simply follow the provided links:

- [husarion/panther](https://hub.docker.com/r/husarion/panther) - ROS packages for Panther robot,
- [husarion/panther-gazebo](https://hub.docker.com/r/husarion/panther-gazebo) - Simulation for Panther robot in Gazebo-classic.

### Hardware image layout

The hardware image (`husarion/husarion-ugv`) holds the built install space in `/ros2_ws/install` and its runtime dependencies. It has no `/ros2_ws/src`, no `/ros2_ws/build`, no C toolchain and no build-only dependencies, and the install space is not symlinked into the sources.

To change a launch or config file of the driver without rebuilding, bind-mount it over its copy in `/ros2_ws/install/<package>/share/<package>/`. A mount over `/ros2_ws/src` has no effect.

To build your own packages against the driver, use the `builder` stage of [Dockerfile.hardware](./Dockerfile.hardware) (`docker build --target builder -f docker/Dockerfile.hardware .` from the repository root) or the simulation image. The `builder` stage is a build environment only: it lacks the entrypoint setup of the robot environment, so run the driver from the full image.

A local build keeps the dependency layer, including its `apt-get upgrade`, cached as long as the package manifests do not change. Pass a new `--build-arg APT_CACHE_BUST=<value>` (for example the date) to rebuild it with current packages.

## Updating Panther Software

> [!Note]
> Latest Panther Docker images are compatible with Built-in Computer OS version 2.0.0 and newer. If your operating system is older, please ensure you update it before proceeding. **[COMING SOON]** Follow [operating system reinstallation](ros2-os-instalation-link) for more info.

Connect to Panther's Built-in Computer:

```bash
ssh husarion@10.15.20.2
```

## Quick Start

Depending on your needs, you can run Docker to quickly launch the physical robot or run a simulation. To do this, clone this repository to your robot or computer.

```bash
cd docker/demo
```

### 🤖 Robot

On every start, the container copies the driver configuration files that are missing from `/config`, for example on the first run, after the robot model changed, or when a driver update added a file. This start-up copy never overwrites a file that exists, so a file you deleted from `/config` comes back on the next start, while your edits stay.

1. (Optional) Updating configuration files.

   > [!NOTE]
   > The command below refreshes every file and may overwrite your changes made in the `config` directory. If you want to keep your configuration you should skip this step or create backup of the `config` directory.

   ```bash
   docker run --rm -v /home/husarion/config:/config husarion/husarion-ugv:jazzy-<newest_tag> update_config_directory
   ```

2. Activate Panther

   ```bash
   docker compose -f compose.hardware.yaml up
   ```

   Extra bringup launch arguments can be passed in the `BRINGUP_EXTRA_ARGS` variable. They are appended to the default arguments, and the last value given for an argument wins, so they can also override a default such as `launch_gamepad:=false`:

   ```bash
   BRINGUP_EXTRA_ARGS="log_level:=WARN wheel_type:=WH05" docker compose -f compose.hardware.yaml up
   ```

   Compose reads the variable from the shell or from a `.env` file next to the compose file (a line `BRINGUP_EXTRA_ARGS=log_level:=WARN wheel_type:=WH05`), not from the service's `env_file`.

3. Launch Visualization on PC

   ```bash
   xhost local:root
   docker compose -f compose.rviz.yaml up
   ```

> [!NOTE]
> To use the latest version of the image, run the `docker compose pull` command and rerun above commands.

### 💻 Gazebo Simulation

To give Docker access to your screen run:

```bash
xhost local:docker
docker compose -f compose.simulation.yaml up
```

> [!NOTE]
>
> 1. You can change robot model and namespace by editing the launch command in `compose.simulation.yaml`.
> 2. If you have an NVIDIA GPU, it is worth changing the compose configuration from `cpu-config` to `gpu-config`. For this purpose, it is necessary to install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). With NVIDIA Container Toolkit installed, modify following Docker compose file by replacing `*cpu-config` with `*gpu-config`: [compose.simulation.yaml](./demo/compose.simulation.yaml).
