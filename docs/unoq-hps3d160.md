# UNO Q Linux-side utilities (USB host mode + HPS-3D160)
Separate from the LD19 ROS 2 driver setup in [`../README.md`](../README.md),
this repo also tracks a few files used to configure the UNO Q's own Linux side
so it can act as a USB host for an attached HPS-3D160-U lidar (a different
sensor from the LD19, used here to validate the USB-C host-mode wiring):
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

## Real-time point cloud streaming and visualization
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
  red=near/violet=far, with alternating brightness between bands for
  contrast), redrawn on a fixed Qt timer independent of the
  sensor's own frame rate. Can also record the stream to disk (see
  Recording and replaying captures below).
- `pointcloud_replay.py` — plays a recording back in the same 3D view,
  with no sensor, board, or network connection needed.

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

Both `hps3d160_stream` and `pointcloud_viewer.py` auto-reconnect (see
code comments), so they can be started/restarted in either order.

### Axis orientation defaults
`remap_for_display()` (shared by both tools) turns the sensor's raw XYZ
into a display-friendly orientation. It has no way to know the sensor's
actual physical mounting on its own, so it exposes four flags to correct
for it: `--swap-xy`, `--mirror-lr`, `--invert-vertical`, and `--rotate`.

As shipped, these default to the mounting verified empirically for this
setup (move to a known physical side of the sensor and check where the
point renders): `--swap-xy`, `--mirror-lr`, and `--invert-vertical` all
default **on**, and `--rotate` defaults to **25** degrees. If you're
running a differently-mounted HPS-3D160, override with the boolean
flags' `--no-` form, and/or a different `--rotate` value:
```bash
python3 pointcloud_viewer.py --port 5555 \
  --no-swap-xy --no-mirror-lr --no-invert-vertical --rotate 0
```
- `--swap-xy` / `--no-swap-xy`: swaps which native axis feeds
  horizontal vs. vertical on screen -- needed if the sensor is mounted
  rolled 90 degrees from what's assumed (left/right motion shows up as
  vertical motion, or vice versa). No combination of the other three
  flags can fix this, since they only flip signs or yaw in the
  horizontal/depth plane.
- `--mirror-lr` / `--no-mirror-lr`: flips which physical side renders as
  "left".
- `--invert-vertical` / `--no-invert-vertical`: flips the vertical axis
  if the scene renders upside down.
- `--rotate DEGREES` (default 25; not boolean, just pass a different
  value): yaws the scene around the vertical axis, for a sensor bolted
  facing a direction other than "straight ahead".
- `--invert-depth` (default off, still a plain flag): negates depth if
  it comes out backwards after `--rotate`; see `remap_for_display()`'s
  docstring in `pointcloud_viewer.py` for why this is a separate flag
  from `--rotate` rather than just another angle.

### Auto-starting the streamer on boot
`hps3d160-stream.service` runs `hps3d160_stream` automatically on the
UNO Q, after `usb-role-host.service` and networking are up, and restarts
it if it ever exits (e.g. an actual sensor disconnect):
```bash
scp hps3d160-stream.service arduino@<board-ip>:~/
ssh arduino@<board-ip> '\
  sudo mv ~/hps3d160-stream.service /etc/systemd/system/ && \
  sudo systemctl daemon-reload && \
  sudo systemctl enable --now hps3d160-stream.service'
```
The unit hardcodes the viewer's host/port (`192.168.1.169 5555`) in its
`ExecStart` line — update that and run `sudo systemctl restart
hps3d160-stream.service` if either changes. It also sets
`WorkingDirectory` to the build directory, since `hps3d160_stream` was
linked with a relative rpath (`-rpath=./`) that only resolves against
`libHPS3DSDK.so` if the process's cwd matches; without this, systemd
fails the unit with an `error while loading shared libraries` / exit
code 127.

With this service running, only `pointcloud_viewer.py` needs to be
started manually after a power cycle.

### Recording and replaying captures
Pass `--record` to save the session to a compressed NumPy archive while
the live view runs as usual, then replay it later with
`pointcloud_replay.py`:
```bash
# capture (Ctrl+C to stop and finalize the file)
python3 pointcloud_viewer.py --port 5555 --record capture.npz

# replay, no sensor or board required
python3 pointcloud_replay.py capture.npz --fps 30
```
Replay controls: space pauses/resumes, left/right steps a single frame,
home/end jumps to the first/last frame; the mouse orbits/zooms/pans as in
the live viewer. Playback holds on the final frame unless `--loop` is
given. The rendering flags (`--fps`, `--point-size`, `--bands`,
`--edge-filter-mm`, `--invert-vertical`, `--mirror-lr`, `--swap-xy`,
`--rotate`, `--invert-depth`, `--axis-size`) and the scene/camera flags
(`--grid-size`, `--grid-spacing`, `--camera-distance`, `--elevation`,
`--azimuth`) behave identically in both tools, including their defaults
(see Axis orientation defaults above). They're defined once, in
`add_visualization_args()` in `pointcloud_viewer.py`, which
`pointcloud_replay.py` imports along with the color-banding and
axis-remapping helpers rather than duplicating them -- so the two can't
drift apart on the mirroring subtleties documented in
`remap_for_display()`, nor on a default silently changed in only one
place. It does mean both files need to sit side by side.

Only genuinely tool-specific flags live in each script: `--host`,
`--port`, and `--record` for the live viewer, `--loop` and
`--status-height` (height of the replay's frame-counter label, default
2.5x `--axis-size`) for the replay.

What gets saved, and the file layout:
- **Every frame received off the socket** is recorded, not just the ones
  the ~30fps render timer happens to draw. Recording therefore captures
  the sensor's true output rate.
- A background writer thread fed by a bounded queue does the compression,
  so disk I/O can never stall the socket reader. If the writer falls
  behind, frames are dropped rather than backpressuring the sensor, and
  the count is printed at shutdown.
- Frames stream into the archive one at a time instead of accumulating in
  RAM — at ~115 kB per frame before compression, buffering a long capture
  would quickly reach gigabytes. Replay is lazy for the same reason: it
  decompresses one frame at a time, so a large recording plays back in
  roughly one frame's worth of memory.
- Members are `frame_000000`, `frame_000001`, ... each a `(height, width,
  3)` float32 array of XYZ in mm, in the sensor's native axes (the grid
  layout is preserved rather than flattened). A `frame_info` member holds
  `(frame_cnt, width, height)` per frame; replay uses the counter to
  report gaps caused by dropped frames, so a discontinuity in the data
  isn't mistaken for something moving in the scene.

No special tooling is needed to read a capture — it's a plain `.npz`:
```python
data = np.load("capture.npz")
xyz = data["frame_000042"]   # (60, 160, 3) float32, mm, sensor axes
```

> **Caveat:** a `.npz` is a zip, and its central directory is only written
> on close. Ctrl+C and `SIGTERM` finalize the archive cleanly, but a
> `SIGKILL` or power loss leaves the file unreadable *in its entirety*,
> not merely truncated. Don't `kill -9` a long capture you care about.

### Depth-quality filtering
Two point-level filters run before color banding, in both `pointcloud_viewer.py`
and `pointcloud_replay.py` (shared via `make_banded_colors()`):
- **No-return sentinel rejection.** The HPS-3D160 reports a fixed,
  physically implausible depth (~65300-65500mm, near the max 16-bit
  millimeter value) for pixels with no valid return, rather than marking
  them invalid outright -- and it can be a sizeable minority of a frame.
  Left in, that sentinel would dominate the near/far range used for color
  banding and crush all real scene depth into a couple of bands
  (near-monochrome). It's excluded via a fixed 20m cutoff rather than a
  statistical (e.g. median-based) outlier filter: a statistical filter
  can't distinguish "sentinel" from "real object that's just far from
  everything else in frame" -- a scene dominated by a nearby, uniform
  wall has a tiny statistical spread, so a real person standing further
  away would read as a huge outlier and get wrongly hidden entirely. A
  fixed physical cutoff, safely between any real indoor reading and the
  sentinel, has no such failure mode.
- **Flying-pixel edge filter (`--edge-filter-mm`, default 300, 0
  disables).** Pixels straddling an object's silhouette see a mix of
  light from the object and from whatever is behind it, and report a
  blended depth between the two rather than either real surface. On a
  moving object the blend ratio drifts frame to frame as the edge sweeps
  across each pixel, which reads as edges looking "liquid"/fuzzy rather
  than crisp. These are detected by comparing each pixel's depth to its
  immediate neighbors in the sensor's scan grid (row/column-adjacent, not
  3D-adjacent) -- a real continuous surface changes gradually between
  neighbors, while a flying pixel jumps sharply toward the surface behind
  it. This needs the frame's width/height to know which points are
  grid-adjacent; `pointcloud_replay.py` gets it for free from each stored
  frame's `(height, width, 3)` shape, while the live viewer's
  `FrameReceiver` tracks it alongside each frame's points.

## Restoring after an App Lab OS re-flash
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
