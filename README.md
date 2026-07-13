# LD19 LiDAR — ROS 2 (Jazzy) Docker setup for Arduino UNO Q

Runs the maintained ROS 2 driver
[`ldrobotSensorTeam/ldlidar_stl_ros2`](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2)
in a container **targeting the Arduino UNO Q**. Package name: `ldlidar_stl_ros2`;
scan is published on `/scan` with frame `base_laser`.

> Note: the originally cloned `DFRobotdl/ldlidar_stl_ros` is the **ROS 1 (catkin)**
> version and does not build under ROS 2. This setup uses the ROS 2 port instead.

## Target platform (important)
The UNO Q runs Debian Linux on a **64-bit Arm Cortex-A53 (`arm64`)** MPU and
supports Docker + Docker Compose. The image is therefore built for
`linux/arm64` (pinned in `docker-compose.yml`). The LiDAR plugs into the UNO Q,
where its CP2102 adapter enumerates as `/dev/ttyUSB0`.

There are two ways to get the image onto the board:

### Path A — build on the UNO Q (recommended)
Native `arm64` build, no emulation. Copy this folder to the board and build there:
```bash
# from your dev machine
scp -r ldlidar_ros2_docker <user>@<uno-q-host>:~/
# on the UNO Q
cd ~/ldlidar_ros2_docker
docker compose build
```
The A53 is modest, so expect the `colcon build` step to take a while.

### Path B — cross-build on an x86_64 host, then transfer
Requires a one-time privileged QEMU registration, then use the helper script:
```bash
# one-time: enable arm64 emulation on the x86_64 host
docker run --privileged --rm tonistiigi/binfmt --install arm64
# cross-build + export a tarball
./build-arm64.sh
# copy to the board and load
scp ldlidar_stl_ros2_jazzy_arm64.tar <user>@<uno-q-host>:~/
ssh <user>@<uno-q-host> docker load -i ~/ldlidar_stl_ros2_jazzy_arm64.tar
```
Emulated compilation is significantly slower than Path A.

## Prerequisites (on the board)
- Docker with the Compose plugin (`docker compose`) — preinstalled on UNO Q.
- The LiDAR connected via its CP2102 USB-serial adapter (usually `/dev/ttyUSB0`).

## Pinned driver version
The Dockerfile pins the driver to the upstream release tag **`v3.0.3`** for
reproducible builds (`LDLIDAR_BRANCH` arg). To build a different tag/branch
without editing the file:
```bash
docker compose build --build-arg LDLIDAR_BRANCH=v2.3.0
```

## Run

### 1. Grant access to the serial device (on the UNO Q)
```bash
ls -l /dev/ttyUSB*          # confirm the device node
sudo chmod 666 /dev/ttyUSB0 # or add your user to the 'dialout' group
```
If your device is not `/dev/ttyUSB0`, update the `devices:` mapping in
`docker-compose.yml`.

### 2a. Launch the driver only (headless)
```bash
docker compose run --rm ldlidar \
  ros2 launch ldlidar_stl_ros2 ld19.launch.py
```

### 2b. Launch driver + RViz2 visualization
RViz needs a display. Either attach a monitor to the UNO Q (SBC mode) and run
the viewer there, or run only the driver on the board and visualize from
another ROS 2 machine on the same network (see DDS note below). To show RViz on
whichever machine has the display, allow the container to reach its X server:
```bash
xhost +local:root
docker compose run --rm ldlidar \
  ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
```

### Interactive shell
```bash
docker compose run --rm ldlidar bash
# inside the container the workspace is already sourced:
ros2 launch ldlidar_stl_ros2 ld19.launch.py
ros2 topic echo /scan
```

## Notes
- **Serial port name**: set the LiDAR port in
  `src/ldlidar_stl_ros2/launch/ld19.launch.py` (`port_name`). The default is
  `/dev/ttyUSB0`. To change it without rebuilding, mount an edited launch file
  or edit and `colcon build` inside the container.
- **DDS discovery**: the compose file uses `network_mode: host` so topics are
  visible to other ROS 2 nodes on the LAN. This lets you run the driver on the
  UNO Q and RViz2 on a separate workstation.
- **Display / X11**: RViz2 requires a display on whichever machine runs it.
  `xhost +local:root` plus the mounted `/tmp/.X11-unix` is usually enough; if
  the window fails to open, confirm `echo $DISPLAY` is set before
  `docker compose run`. (On a Wayland desktop, X11 apps run via XWayland.)
- **Build patch**: the upstream `ldlidar_driver/src/logger/log_module.cpp` uses
  `pthread_mutex_*` without including `<pthread.h>`, which fails to compile on
  Ubuntu 24.04 / GCC 13. The Dockerfile injects the missing include after
  cloning, so no manual edit is required.
