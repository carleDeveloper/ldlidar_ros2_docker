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

## Deployment walkthrough (cross-build from an x86_64 host)
End-to-end steps used to deploy to the UNO Q. These assume an SSH alias `uno-q`
in `~/.ssh/config` (`HostName 192.168.1.50`, `User arduino`).
1. One-time: enable arm64 emulation on the x86_64 host (does not always survive
   a reboot — re-run if `exec format error` reappears):
   ```bash
   docker run --privileged --rm tonistiigi/binfmt --install arm64
   ```
2. Cross-build the arm64 image (slow under emulation):
   ```bash
   docker compose build
   ```
3. Export the image to a tarball:
   ```bash
   docker save ldlidar_stl_ros2:jazzy-arm64 -o ldlidar_stl_ros2_jazzy_arm64.tar
   ```
4. Transfer to the board and load it into the board's Docker:
   ```bash
   scp ldlidar_stl_ros2_jazzy_arm64.tar uno-q:~/
   ssh uno-q docker load -i ~/ldlidar_stl_ros2_jazzy_arm64.tar
   ```
5. Verify native execution on the board (no LiDAR required):
   ```bash
   ssh uno-q 'docker run --rm ldlidar_stl_ros2:jazzy-arm64 \
     bash -lc "uname -m; ros2 pkg executables ldlidar_stl_ros2"'
   ```
   Expect `aarch64` and the two executables (`ldlidar_stl_ros2`,
   `ldlidar_stl_ros2_node`) with no `exec format error`.
6. Remove the tarball once loaded (see Cleanup below).

The `build-arm64.sh` helper wraps steps 2–3 (build + save) once emulation from
step 1 is in place.

## Cleanup
The exported image tarball is large (~1.3 GB) and only needed for transfer.
After the image is loaded on the board, remove both copies:
```bash
rm -f ldlidar_stl_ros2_jazzy_arm64.tar          # on the dev host
ssh uno-q rm -f ~/ldlidar_stl_ros2_jazzy_arm64.tar  # on the board
```
`*.tar` is git-ignored, so tarballs are never committed.

## Build verification (native x86_64 host)
Verified the current `Dockerfile` still builds cleanly and the resulting
image runs correctly on a native x86_64 dev machine (no QEMU needed for this
check, unlike the arm64 path above):
```bash
docker build -t ldlidar_stl_ros2:jazzy .
```
Build completed with only a pre-existing upstream compiler warning
(uninitialized `serial_port_baudrate` in `demo.cpp`), no errors.

Confirmed the package and its executables are present:
```bash
docker run --rm ldlidar_stl_ros2:jazzy bash -lc \
  "uname -m; ros2 pkg executables ldlidar_stl_ros2"
```
Output: `x86_64`, with both executables listed (`ldlidar_stl_ros2`,
`ldlidar_stl_ros2_node`) — the x86_64 equivalent of the `aarch64` check in
step 5 of the deployment walkthrough above.

Also ran the node directly with no launch parameters, to confirm the binary
and its shared libraries actually load (not just that the CLI lists them):
```bash
docker run --rm ldlidar_stl_ros2:jazzy bash -lc \
  "source /opt/ros/jazzy/setup.bash; source /ros2_ws/install/setup.bash; \
   ros2 run ldlidar_stl_ros2 ldlidar_stl_ros2_node"
```
It initializes, logs `LDLiDAR SDK Pack Version is: v3.0.3` (matching the
pinned tag), and exits with an expected application-level error
(`Error, input <product_name> is illegal`) since no launch parameters were
supplied — confirming the node itself runs correctly, as opposed to a
missing-library or exec-format failure. Testing against real LiDAR hardware
still requires the appropriate `port_name`/`product_name` launch parameters.

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

## UNO Q Linux-side utilities (USB host mode + HPS-3D160 verification)
Separate from the ROS 2 driver above, this repo also tracks a few files used
to configure the UNO Q's own Linux side so it can act as a USB host for an
attached HPS-3D160-U lidar (a different sensor from the LD19, used here to
validate the USB-C host-mode wiring):
- `usb-role-host.service` — a systemd oneshot unit that forces the shared
  USB-C port into host mode on every boot, by writing `host` to
  `/sys/class/usb_role/4e00000.usb-role-switch/role`.
- `verify_hps3d160.py` — sends the HPS-3D160's documented "read device
  address" command over `/dev/ttyACM0` and checks the response, to confirm
  the lidar link is alive.
- `backup_unoq_config.sh` — run with `sudo` on the board **before** an App
  Lab OS re-flash. Archives the two files above, SSH `authorized_keys`, and
  Wi-Fi connection profiles into `/tmp/unoq-backup-<timestamp>.tar.gz`.
  Copy it off the board afterward; it contains the Wi-Fi password in
  plaintext, so it's git-ignored and must never be committed.

### Real-time point cloud streaming and visualization
Builds a live 3D point cloud from the HPS-3D160-U and displays it on a
separate machine's monitor, using Hypersen's official MIT-licensed
[`hypersen/HPS3D_SDK`](https://github.com/hypersen/HPS3D_SDK) on the board
(no ROS2 install needed) and a lightweight TCP stream to a desktop viewer:
- `hps3d160_stream.c` / `Makefile.hps3d160_stream` — runs on the UNO Q.
  Uses the SDK to start continuous capture and streams each frame's
  already-computed XYZ point cloud (`HPS3D_PerPointCloudData_t`, no manual
  FOV/angle math needed) over a plain TCP socket, as a raw little-endian
  binary frame: `[points:u32][width:u16][height:u16][frame_cnt:u32][xyz
  float32 x points]`.
- `pointcloud_viewer.py` — runs on a machine with a display (e.g. this
  desktop). Listens for the board's TCP connection and renders the point
  cloud with `pyqtgraph`'s OpenGL `GLScatterPlotItem` (color-graded
  red=near/blue=far), redrawn on a fixed Qt timer independent of the
  sensor's own frame rate.

Setup (one-time, on the UNO Q):
```bash
sudo apt-get install -y build-essential
git clone --depth 1 https://github.com/hypersen/HPS3D_SDK.git
cp HPS3D_SDK/V1.8/Example/HPS3D160-Linux-C_Demo/HPS3DUser_IF.h \
   HPS3D_SDK/V1.8/Example/HPS3D160-Linux-C_Demo/HPS3DBase_IF.h \
   HPS3D_SDK/V1.8/lib/Linux/libHPS3DSDK_aarch64_1-8-6.so ./
cp libHPS3DSDK_aarch64_1-8-6.so libHPS3DSDK.so
make -f Makefile.hps3d160_stream
```
(Pick the correct architecture-specific `.so` under `lib/Linux/` if not
aarch64 — the SDK ships `x86_64`, `arm32`, `arm64`/`aarch64` builds.)

Run (viewer first, then the board):
```bash
# on the display machine
python3 pointcloud_viewer.py --port 5555 --fps 30

# on the UNO Q
./hps3d160_stream <viewer-host-ip> 5555 /dev/ttyACM0
```
Requires `python-pyqtgraph`, `python-pyqt6`, `python-numpy`, and
`python-opengl` (all in Arch's official `extra` repo) on the viewer
machine.

Note: 30fps is the viewer's *render* target, not a guarantee of the
sensor's actual output rate — in testing the sensor itself sustained
~13.6fps (likely HDR/integration-time limited under ambient lighting),
while the viewer smoothly redraws the latest available frame at ~30fps
regardless. If a higher native sensor frame rate matters, check the
device's HDR/exposure settings via the vendor's Windows client software.

### Restoring after an App Lab OS re-flash
An App Lab "Flash Board" operation wipes the eMMC, so all of the above is
lost: SSH host keys regenerate, `authorized_keys` is gone, and the systemd
unit/script no longer exist. To restore (assuming Wi-Fi is already
reconnected via App Lab's first-boot wizard, so the board is reachable at
its usual IP):
1. Clear the stale SSH host key and re-authenticate:
   ```bash
   ssh-keygen -R <board-ip>
   ssh-copy-id arduino@<board-ip>   # prompts for the board's password
   ```
2. Copy the two files back from this repo and reinstall the service:
   ```bash
   scp usb-role-host.service verify_hps3d160.py arduino@<board-ip>:~/
   ssh arduino@<board-ip> '\
     sudo mv ~/usb-role-host.service /etc/systemd/system/usb-role-host.service && \
     sudo systemctl daemon-reload && \
     sudo systemctl enable --now usb-role-host.service'
   ```
3. Verify it took effect:
   ```bash
   ssh arduino@<board-ip> '\
     cat /sys/class/usb_role/4e00000.usb-role-switch/role; \
     python3 ~/verify_hps3d160.py /dev/ttyACM0'
   ```
   Expect `host` and a response frame starting with `f5 5f`.

Everything needed for this recovery already lives in git — the pre-reflash
backup tarball from `backup_unoq_config.sh` is a convenience/fallback, not a
requirement, since `usb-role-host.service` and `verify_hps3d160.py` are
already tracked here.
