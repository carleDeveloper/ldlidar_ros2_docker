#!/usr/bin/env python3
"""
Real-time point cloud viewer for the HPS-3D160-U lidar.

Runs on a machine with a display. Listens for a single TCP connection from
`hps3d160_stream` (running on the Arduino UNO Q's Linux side), which sends
one binary frame per capture:

    uint32 points      (e.g. 9600 = 160*60)
    uint16 width        (e.g. 160)
    uint16 height        (e.g. 60)
    uint32 frame_cnt
    float32 xyz[points][3]   (all little-endian; no byte-swapping needed
                              between aarch64 and x86_64)

A background thread reads frames off the socket as fast as they arrive and
keeps only the latest one; a Qt timer redraws the 3D scatter view at a
fixed interval (default ~30fps) using whatever the latest frame is. This
decouples network throughput from render rate, so a slow render doesn't
back up the socket, and a fast network doesn't force excess redraws.

Pass --record FILE.npz to also save every received frame (not just the
rendered ones) to a compressed NumPy archive. Reload it later with:

    data = np.load("FILE.npz")
    info = data["frame_info"]        # (n, 3) int64: frame_cnt, width, height
    first = data["frame_000000"]     # (height, width, 3) float32 xyz, in mm

Usage:
    python3 pointcloud_viewer.py [--host 0.0.0.0] [--port 5555] [--fps 30]
                                 [--record capture.npz]
"""
import argparse
import colorsys
import io
import queue
import signal
import socket
import struct
import sys
import threading
import zipfile

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

FRAME_HEADER_FMT = "<IHHI"  # points(u32), width(u16), height(u16), frame_cnt(u32)
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FMT)

# Fraction of the hue wheel used for depth banding: 0.0=red, up to ~0.85=violet.
# Stops short of 1.0 so the far end (violet) stays visually distinct from the
# near end (red) instead of wrapping back around to it.
_BAND_HUE_SWEEP = 0.85
# Alternate-band brightness (vs. full value=1.0), for boundary contrast between
# neighboring bands whose hues land close together -- see _band_palette().
_BAND_DIM_VALUE = 0.6
# Depth (mm) at/beyond which a reading is treated as the sensor's "no valid
# return" sentinel rather than a real surface -- see make_banded_colors().
# Observed sentinel values are ~65300-65500mm, near the max representable
# 16-bit millimeter value; this HPS-3D160 has a normal indoor working range
# of a few meters, so a fixed cutoff at 20m leaves an order-of-magnitude
# margin on both sides without ever excluding real scene depth.
#
# A per-frame *statistical* outlier filter (e.g. median absolute deviation)
# was tried here first and reverted: it can't tell "sentinel" apart from
# "real object that's simply far from the rest of the frame". A scene
# dominated by a nearby, uniform surface (e.g. a wall or floor) has a tiny
# statistical spread, so a real person standing further away reads as a
# huge outlier relative to that spread and gets hidden entirely -- silently
# erasing exactly the kind of foreground object depth banding exists to
# highlight. A fixed physical cutoff has no such failure mode, since it
# doesn't depend on what else is in the frame.
_NO_RETURN_DEPTH_MM = 20000.0
# Default threshold (mm) for _mask_flying_pixels(): a depth jump larger than
# this between two adjacent pixels in the sensor's scan grid is treated as a
# "flying pixel" -- see that function's docstring. 300mm comfortably exceeds
# the sensor's own measurement noise on a continuous surface, while still
# being far smaller than the near/far gap at a typical object silhouette.
_FLYING_PIXEL_MAX_NEIGHBOR_DIFF_MM = 300.0


class FrameRecorder(threading.Thread):
    """Background thread that appends every submitted frame to a .npz archive.

    A .npz file is just a zip of .npy members, so we can stream frames into it
    one at a time instead of buffering the whole capture in RAM (a 160x60
    frame is ~115 kB, i.e. ~3.5 MB/s at 30fps -- a few minutes of recording
    would otherwise be gigabytes).

    Writing happens on this thread, fed by a bounded queue, so disk I/O and
    compression can never stall the socket reader. If the writer can't keep
    up, frames are dropped rather than applying backpressure to the sensor.
    """

    _SENTINEL = object()

    def __init__(self, path: str, queue_size: int = 120):
        super().__init__(daemon=True)
        self.path = path
        self._queue = queue.Queue(maxsize=queue_size)
        self.frames_written = 0
        self.frames_dropped = 0

    def submit(self, xyz: np.ndarray, width: int, height: int, frame_cnt: int):
        try:
            self._queue.put_nowait((xyz, width, height, frame_cnt))
        except queue.Full:
            self.frames_dropped += 1

    def close(self, timeout: float = 10.0):
        """Flush queued frames and finalize the archive."""
        self._queue.put(self._SENTINEL)
        self.join(timeout=timeout)

    @staticmethod
    def _write_array(zf: zipfile.ZipFile, name: str, arr: np.ndarray):
        buf = io.BytesIO()
        np.lib.format.write_array(buf, arr, allow_pickle=False)
        zf.writestr(name, buf.getvalue())

    def run(self):
        frame_info = []
        try:
            # compresslevel=1: these are float32 depth samples, so heavier
            # deflate settings cost a lot of CPU for very little size win.
            with zipfile.ZipFile(self.path, "w", zipfile.ZIP_DEFLATED,
                                 allowZip64=True, compresslevel=1) as zf:
                while True:
                    item = self._queue.get()
                    if item is self._SENTINEL:
                        break
                    xyz, width, height, frame_cnt = item
                    if width * height == len(xyz):
                        xyz = xyz.reshape(height, width, 3)  # keep the sensor's grid layout
                    self._write_array(zf, f"frame_{self.frames_written:06d}.npy", xyz)
                    frame_info.append((frame_cnt, width, height))
                    self.frames_written += 1
                self._write_array(zf, "frame_info.npy",
                                  np.array(frame_info, dtype=np.int64).reshape(-1, 3))
        except OSError as exc:
            print(f"Recording to {self.path} failed: {exc}")


class FrameReceiver(threading.Thread):
    """Background thread: accepts one connection and keeps the latest frame."""

    def __init__(self, host: str, port: int, recorder: "FrameRecorder | None" = None):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.recorder = recorder
        self._lock = threading.Lock()
        self._latest_points = None  # np.ndarray shape (N, 3), or None
        self._width = None
        self._height = None
        self._frame_cnt = None
        self.connected_event = threading.Event()
        self.stop_event = threading.Event()

    def get_latest(self):
        with self._lock:
            return self._latest_points, self._width, self._height, self._frame_cnt

    def _recv_exact(self, conn: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("connection closed")
            buf.extend(chunk)
        return bytes(buf)

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(1.0)  # let accept() re-check stop_event periodically
        print(f"Listening on {self.host}:{self.port} for the board's stream...")

        try:
            while not self.stop_event.is_set():
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    print(f"accept() failed: {exc}; retrying...")
                    continue

                print(f"Connected: {addr}")
                # Bound recv() so a dead peer that never sends a clean TCP close
                # (e.g. power loss, network partition) doesn't block forever --
                # we want to notice and go back to waiting for a new connection.
                conn.settimeout(5.0)
                self.connected_event.set()
                try:
                    while not self.stop_event.is_set():
                        header = self._recv_exact(conn, FRAME_HEADER_SIZE)
                        points, width, height, frame_cnt = struct.unpack(FRAME_HEADER_FMT, header)
                        payload = self._recv_exact(conn, points * 3 * 4)
                        xyz = np.frombuffer(payload, dtype="<f4").reshape(points, 3)
                        with self._lock:
                            self._latest_points = xyz
                            self._width = width
                            self._height = height
                            self._frame_cnt = frame_cnt
                        if self.recorder is not None:
                            self.recorder.submit(xyz, width, height, frame_cnt)
                except socket.timeout:
                    print("No data from board for 5s, assuming it's gone; waiting for a new connection...")
                except (ConnectionError, OSError, struct.error, ValueError) as exc:
                    print(f"Board disconnected ({exc}), waiting for a new connection...")
                finally:
                    self.connected_event.clear()
                    conn.close()
        finally:
            srv.close()


def _band_palette(num_bands: int) -> np.ndarray:
    """One color per band: hue sweeps red (band 0, near) -> violet (last
    band, far) across most of the color wheel, like a contour-map legend.

    Two things make bands easier to tell apart than a plain hue ramp:
    - The sweep covers ~306 degrees (red through violet) rather than
      stopping at blue, so there's more hue range to spread bands across
      -- this matters most once --bands grows past ~10 and neighboring
      bands would otherwise land on nearly-identical hues.
    - Adjacent bands alternate brightness (full vs. dimmed). Even with
      the wider sweep, two neighboring bands can still end up close in
      hue; alternating brightness gives every band boundary a second,
      hue-independent cue, the way contour maps alternate shading between
      neighboring elevation bands.
    """
    palette = np.zeros((num_bands, 3), dtype=np.float32)
    for i in range(num_bands):
        t_band = i / max(num_bands - 1, 1)
        hue = t_band * _BAND_HUE_SWEEP
        value = 1.0 if i % 2 == 0 else _BAND_DIM_VALUE
        palette[i] = colorsys.hsv_to_rgb(hue, 1.0, value)
    return palette


def _mask_flying_pixels(z_grid: np.ndarray, max_diff_mm: float) -> np.ndarray:
    """Given depth shaped (height, width) -- NaN for already-invalid pixels
    -- return a same-shape boolean mask, True where a pixel is *not* a
    "flying pixel".

    Depth-camera pixels straddling the silhouette of an object see a mix of
    light returning from that object and from whatever is behind it, and
    report a blended depth somewhere between the two rather than either
    real surface. This is normally a minor, static fringe of bad pixels
    right at an edge, but on a *moving* object the blend ratio drifts
    frame to frame as the edge sweeps across each pixel, so the reported
    depth (and therefore the point's rendered position/band color) wobbles
    between the near and far surface from frame to frame -- which is what
    reads as edges "liquid"/fuzzy rather than a crisp boundary.

    Flying pixels are a real return, not the sensor's explicit no-return
    sentinel, so they aren't caught by the >0/isfinite/_NO_RETURN_DEPTH_MM
    checks above. What does identify them is that they sit between two
    real surfaces at very different depths, so they show a large depth
    jump to an immediate grid neighbor that a continuous surface wouldn't.
    Pixels already NaN (invalid for other reasons) can't be compared
    meaningfully; NaN comparisons are False in numpy, so they're correctly
    treated as "unknown" here rather than falsely flagged as edges.
    """
    ok = np.ones(z_grid.shape, dtype=bool)
    diff_h = np.abs(np.diff(z_grid, axis=1)) > max_diff_mm
    ok[:, :-1] &= ~diff_h
    ok[:, 1:] &= ~diff_h
    diff_v = np.abs(np.diff(z_grid, axis=0)) > max_diff_mm
    ok[:-1, :] &= ~diff_v
    ok[1:, :] &= ~diff_v
    return ok


def make_banded_colors(z: np.ndarray, num_bands: int, width: int = None, height: int = None,
                       edge_filter_mm: float = _FLYING_PIXEL_MAX_NEIGHBOR_DIFF_MM) -> np.ndarray:
    """Quantize depth (Z) into `num_bands` discrete color bands (near=red,
    far=violet), instead of a smooth gradient, so distance steps are
    visually distinct (like contour-map bands).

    The HPS-3D160 emits a fixed, physically implausible depth (observed:
    ~65300-65500mm, see _NO_RETURN_DEPTH_MM) for "no valid return" pixels
    rather than marking them invalid outright, and it can be a sizeable
    minority of a frame (seen up to ~20%). Left in, those points
    single-handedly dominate the near/far range the bands are stretched
    across, crushing all *real* scene depth into the first band or two --
    the scene ends up looking almost monochrome regardless of --bands.
    Excluding readings at/beyond that cutoff, the same way already-invalid
    (<=0 or non-finite) points are excluded, keeps the near/far range tied
    to genuine surfaces.

    If `width`/`height` are given and match `len(z)`, also hides "flying
    pixel" edge artifacts -- see _mask_flying_pixels(). Pass edge_filter_mm
    <= 0 to disable just that part.
    """
    valid = np.isfinite(z) & (z > 0) & (z < _NO_RETURN_DEPTH_MM)
    colors = np.zeros((len(z), 4), dtype=np.float32)
    if not np.any(valid):
        return colors

    if width and height and edge_filter_mm > 0 and width * height == len(z):
        z_grid = np.where(valid, z, np.nan).reshape(height, width)
        edge_ok = _mask_flying_pixels(z_grid, edge_filter_mm).reshape(-1)
        if np.any(valid & edge_ok):  # don't filter everything away
            valid &= edge_ok

    zmin, zmax = z[valid].min(), z[valid].max()
    span = max(zmax - zmin, 1e-6)
    t = np.clip((z - zmin) / span, 0.0, 1.0)
    band_idx = np.clip((t * num_bands).astype(int), 0, num_bands - 1)

    palette = _band_palette(num_bands)
    colors[:, :3] = palette[band_idx]
    colors[:, 3] = 1.0
    colors[~valid] = 0.0  # hide invalid/out-of-range/flying-pixel points (alpha 0)
    return colors


def remap_for_display(xyz: np.ndarray, invert_vertical: bool, mirror_lr: bool,
                      rotate_deg: float = 0.0, invert_depth: bool = False,
                      swap_xy: bool = False) -> np.ndarray:
    """The sensor's native axes are assumed to be X=left/right, Y=vertical,
    Z=depth (forward from the sensor). pyqtgraph's GLViewWidget always
    treats Z as "up" for its orbit camera and for GLGridItem's ground
    plane, so we reorder into plotting axes X=left/right, Y=depth,
    Z=vertical. This makes mouse-drag orbiting behave like a normal 3D
    viewer (rotate around true vertical, tilt up/down) instead of an
    arbitrary axis.

    IMPORTANT: swapping two axes (Y<->Z here) is a single transposition,
    which always reverses handedness -- i.e. it mirrors the scene into its
    mirror image, not just a harmless relabeling. We negate X below to
    compensate and restore a proper (non-mirrored) 3D scene. Without this,
    every previous version of this function was rendering a mirror image.

    `swap_xy` (--swap-xy) handles a *third* possible mismatch, distinct
    from the two ambiguities below: the sensor may be physically mounted
    rolled 90 degrees relative to what the code assumes (e.g. landscape
    vs. portrait), so native X and Y are transposed -- left/right motion
    shows up as vertical motion on screen (or vice versa) instead of a
    sign flip. No amount of --invert-vertical/--mirror-lr/--rotate can fix
    a transposition, since those only flip signs or yaw in the
    horizontal/depth plane; --swap-xy swaps which native axis feeds
    "horizontal" vs. "vertical" before those sign fixes are applied.
    Verify empirically: move to one physical side of the sensor -- if the
    point moves along the vertical (top/bottom) axis on screen instead of
    left/right, restart with --swap-xy, then re-check --invert-vertical /
    --mirror-lr since swapping axes can also flip which flag you need.

    Two genuinely unresolvable-from-code ambiguities remain, since we have
    no ground truth for the sensor's exact mounting/scan-line convention:
    - the sign of the vertical axis (some depth cameras use Y-down like
      image rows) -- toggle with --invert-vertical if it's upside down.
    - which physical side ends up as "left" after the above fix -- toggle
      with --mirror-lr if left/right still doesn't match reality. Verify
      empirically: place an object to one known physical side of the
      sensor and check which side it renders on.

    `rotate_deg` (--rotate) additionally yaws the whole scene around the
    vertical axis by a fixed angle. Unlike the two ambiguities above, this
    isn't a correctness fix -- it's for when the sensor itself is mounted
    facing a different direction than "straight ahead" (e.g. bolted to a
    side wall), so the scene comes in sideways or backwards relative to
    the room. Positive angles rotate counterclockwise viewed from above
    (standard math convention); e.g. rotate_deg=-90 turns what the sensor
    reports as "depth" into "left/right".

    `invert_depth` (--invert-depth) negates the depth axis, applied after
    rotation. This is NOT the same as an extra +/-90 rotate_deg: any pure
    rotation by a multiple of 90 degrees swaps horizontal and depth
    *together with* a negation on one of them (that's what makes it a
    rotation rather than a mirror -- a rotation preserves handedness, a
    swap-with-no-negation reverses it, same as the Y<->Z handedness issue
    described above). If horizontal ends up correct after some rotate_deg
    but depth still comes out backwards, no rotate_deg value can fix that
    alone; --invert-depth supplies the missing mirror.
    """
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    if swap_xy:
        x, y = y, x
    vertical = -y if invert_vertical else y
    horizontal = x if mirror_lr else -x
    depth = z
    if rotate_deg:
        theta = np.radians(rotate_deg)
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        horizontal, depth = (horizontal * cos_t - depth * sin_t,
                             horizontal * sin_t + depth * cos_t)
    if invert_depth:
        depth = -depth
    return np.stack([horizontal, depth, vertical], axis=-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on")
    parser.add_argument("--port", type=int, default=5555, help="TCP port to listen on")
    parser.add_argument("--fps", type=float, default=30.0, help="target render rate")
    parser.add_argument("--point-size", type=float, default=2.0,
                         help="point size in pixels (smaller avoids overlapping points blending into a blob)")
    parser.add_argument("--bands", type=int, default=10,
                         help="number of discrete distance color bands (near=red, far=violet)")
    parser.add_argument("--edge-filter-mm", type=float, default=_FLYING_PIXEL_MAX_NEIGHBOR_DIFF_MM,
                         help="hide 'flying pixel' points whose depth jumps by more than this many "
                              "mm from an adjacent pixel in the sensor's scan grid -- removes the "
                              "liquid/fuzzy blur these cause at moving object edges; 0 disables")
    parser.add_argument("--invert-vertical", action="store_true",
                         help="flip the vertical axis if the scene renders upside down")
    parser.add_argument("--mirror-lr", action="store_true",
                         help="flip left/right if it still doesn't match reality after the built-in mirror fix")
    parser.add_argument("--swap-xy", action="store_true",
                         help="swap the native X/Y axes if left/right motion shows up as vertical motion "
                              "on screen (or vice versa) -- indicates the sensor is mounted rolled 90 "
                              "degrees from what's assumed; re-check --invert-vertical/--mirror-lr after")
    parser.add_argument("--rotate", type=float, default=0.0, metavar="DEGREES",
                         help="yaw the scene by this many degrees around the vertical axis, e.g. if the "
                              "sensor is mounted facing a different direction than assumed")
    parser.add_argument("--invert-depth", action="store_true",
                         help="flip forward/backward if depth still comes out backwards after --rotate "
                              "(a rotation alone can't fix this -- see remap_for_display()'s docstring)")
    parser.add_argument("--axis-size", type=float, default=500.0,
                         help="length (mm) of the XYZ axis gizmo at the sensor's origin")
    parser.add_argument("--record", metavar="FILE.npz",
                         help="save every received frame to a compressed NumPy archive")
    args = parser.parse_args()

    print("Controls: left-drag to orbit, right-drag/scroll to zoom, middle-drag to pan.")
    print("Axis legend at the sensor's origin (0,0,0): "
          "RED = X (left/right), GREEN = depth (forward from sensor), BLUE = vertical"
          + (" [swapped]" if args.swap_xy else "")
          + (" [inverted]" if args.invert_vertical else "")
          + (" [mirrored]" if args.mirror_lr else "") + ".")
    print("If left/right motion on screen shows up as vertical motion (or vice versa), "
          "restart with --swap-xy.")
    print("If the scene looks upside down, restart with --invert-vertical.")
    print("If left/right still doesn't match reality, restart with --mirror-lr.")

    recorder = None
    if args.record:
        recorder = FrameRecorder(args.record)
        recorder.start()
        print(f"Recording every received frame to {args.record}")

    receiver = FrameReceiver(args.host, args.port, recorder)
    receiver.start()

    app = QtWidgets.QApplication(sys.argv)

    # Ctrl+C handling. Qt's event loop blocks inside C code, so Python only
    # gets a chance to run signal handlers when it happens to be executing
    # bytecode -- in practice that's inside a paint callback or our update()
    # timer. A default KeyboardInterrupt raised there is useless: pyqtgraph's
    # GLViewWidget.drawItemTree() wraps every i.paint() in a bare `except:`
    # and merely prints it as "Error while drawing item ...", so the app keeps
    # running and Ctrl+C looks like a random OpenGL rendering failure.
    # Installing an explicit handler means no exception is raised at all: we
    # just ask Qt to leave its event loop and shut down in an orderly way.
    def request_quit(_signum, _frame):
        print("\nShutting down...")
        receiver.stop_event.set()
        app.quit()

    signal.signal(signal.SIGINT, request_quit)
    signal.signal(signal.SIGTERM, request_quit)
    # A periodically-firing no-op timer guarantees the interpreter regains
    # control (and therefore delivers pending signals) even when nothing else
    # is happening -- e.g. before the board ever connects.
    signal_timer = QtCore.QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)

    view = gl.GLViewWidget()
    view.setWindowTitle("HPS-3D160 Point Cloud")
    # Oblique 3/4 view: elevation tilts up/down from the ground plane,
    # azimuth rotates around the vertical axis. Depth increases along +Y
    # (into the grid), vertical is +/-Z (up/down).
    view.setCameraPosition(distance=2000, elevation=20, azimuth=-60)
    view.show()

    grid = gl.GLGridItem()
    grid.setSize(2000, 2000)
    grid.setSpacing(100, 100)
    view.addItem(grid)

    axis = gl.GLAxisItem()
    axis.setSize(args.axis_size, args.axis_size, args.axis_size)
    view.addItem(axis)

    def add_axis_label(pos, text, color):
        label = gl.GLTextItem(pos=pos, text=text, color=color)
        view.addItem(label)

    a = args.axis_size * 1.05
    horizontal_label = "X: left/right (swapped)" if args.swap_xy else "X: left/right"
    add_axis_label([a, 0, 0], horizontal_label, (255, 90, 90, 255))
    add_axis_label([0, a, 0], "depth (fwd)", (90, 255, 90, 255))
    vertical_label = "vertical (inverted)" if args.invert_vertical else "vertical"
    add_axis_label([0, 0, a], vertical_label, (90, 90, 255, 255))

    scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=args.point_size, pxMode=True)
    # Default 'additive' glOptions blends overlapping points into a single
    # bright blob; 'opaque' keeps each point visually distinct.
    scatter.setGLOptions("opaque")
    view.addItem(scatter)

    render_frame_count = 0
    render_window_start = QtCore.QElapsedTimer()
    render_window_start.start()

    def update():
        nonlocal render_frame_count
        xyz, width, height, frame_cnt = receiver.get_latest()
        if xyz is None:
            return
        # color by native depth (sensor Z)
        colors = make_banded_colors(xyz[:, 2], args.bands, width, height, args.edge_filter_mm)
        plotted = remap_for_display(xyz, args.invert_vertical, args.mirror_lr, args.rotate, args.invert_depth,
                                    args.swap_xy)
        scatter.setData(pos=plotted, color=colors, size=args.point_size)

        render_frame_count += 1
        elapsed = render_window_start.elapsed() / 1000.0
        if elapsed >= 1.0:
            print(f"[pointcloud_viewer] rendering ~{render_frame_count / elapsed:.1f} fps "
                  f"(sensor frame_cnt={frame_cnt})")
            render_frame_count = 0
            render_window_start.restart()

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(int(1000 / args.fps))

    exit_code = app.exec()
    receiver.stop_event.set()
    receiver.join(timeout=2.0)
    if recorder is not None:
        recorder.close()
        msg = f"Saved {recorder.frames_written} frames to {args.record}"
        if recorder.frames_dropped:
            msg += f" ({recorder.frames_dropped} dropped: writer couldn't keep up)"
        print(msg)
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
