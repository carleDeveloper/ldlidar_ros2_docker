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

Usage:
    python3 pointcloud_viewer.py [--host 0.0.0.0] [--port 5555] [--fps 30]
"""
import argparse
import colorsys
import socket
import struct
import sys
import threading

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

FRAME_HEADER_FMT = "<IHHI"  # points(u32), width(u16), height(u16), frame_cnt(u32)
FRAME_HEADER_SIZE = struct.calcsize(FRAME_HEADER_FMT)


class FrameReceiver(threading.Thread):
    """Background thread: accepts one connection and keeps the latest frame."""

    def __init__(self, host: str, port: int):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._latest_points = None  # np.ndarray shape (N, 3), or None
        self._frame_cnt = None
        self.connected_event = threading.Event()
        self.stop_event = threading.Event()

    def get_latest(self):
        with self._lock:
            return self._latest_points, self._frame_cnt

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
                        self._frame_cnt = frame_cnt
            except socket.timeout:
                print("No data from board for 5s, assuming it's gone; waiting for a new connection...")
            except (ConnectionError, OSError, struct.error, ValueError) as exc:
                print(f"Board disconnected ({exc}), waiting for a new connection...")
            finally:
                self.connected_event.clear()
                conn.close()


def _band_palette(num_bands: int) -> np.ndarray:
    """One solid RGB color per band, sweeping red (near) -> blue (far)
    through the hue wheel (like a contour-map legend)."""
    palette = np.zeros((num_bands, 3), dtype=np.float32)
    for i in range(num_bands):
        t_band = i / max(num_bands - 1, 1)
        hue = (1.0 - t_band) * 0.66  # 0.66*360=240deg (blue) -> 0deg (red)
        palette[i] = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
    return palette


def make_banded_colors(z: np.ndarray, num_bands: int) -> np.ndarray:
    """Quantize depth (Z) into `num_bands` discrete color bands (near=red,
    far=blue), instead of a smooth gradient, so distance steps are
    visually distinct (like contour-map bands)."""
    valid = np.isfinite(z) & (z > 0)
    colors = np.zeros((len(z), 4), dtype=np.float32)
    if not np.any(valid):
        return colors

    zmin, zmax = z[valid].min(), z[valid].max()
    span = max(zmax - zmin, 1e-6)
    t = np.clip((z - zmin) / span, 0.0, 1.0)
    band_idx = np.clip((t * num_bands).astype(int), 0, num_bands - 1)

    palette = _band_palette(num_bands)
    colors[:, :3] = palette[band_idx]
    colors[:, 3] = 1.0
    colors[~valid] = 0.0  # hide invalid points (alpha 0)
    return colors


def remap_for_display(xyz: np.ndarray, invert_vertical: bool, mirror_lr: bool) -> np.ndarray:
    """The sensor's native axes are X=left/right, Y=vertical, Z=depth
    (forward from the sensor). pyqtgraph's GLViewWidget always treats Z as
    "up" for its orbit camera and for GLGridItem's ground plane, so we
    reorder into plotting axes X=left/right, Y=depth, Z=vertical. This
    makes mouse-drag orbiting behave like a normal 3D viewer (rotate
    around true vertical, tilt up/down) instead of an arbitrary axis.

    IMPORTANT: swapping two axes (Y<->Z here) is a single transposition,
    which always reverses handedness -- i.e. it mirrors the scene into its
    mirror image, not just a harmless relabeling. We negate X below to
    compensate and restore a proper (non-mirrored) 3D scene. Without this,
    every previous version of this function was rendering a mirror image.

    Two genuinely unresolvable-from-code ambiguities remain, since we have
    no ground truth for the sensor's exact mounting/scan-line convention:
    - the sign of the vertical axis (some depth cameras use Y-down like
      image rows) -- toggle with --invert-vertical if it's upside down.
    - which physical side ends up as "left" after the above fix -- toggle
      with --mirror-lr if left/right still doesn't match reality. Verify
      empirically: place an object to one known physical side of the
      sensor and check which side it renders on.
    """
    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]
    vertical = -y if invert_vertical else y
    horizontal = x if mirror_lr else -x
    return np.stack([horizontal, z, vertical], axis=-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on")
    parser.add_argument("--port", type=int, default=5555, help="TCP port to listen on")
    parser.add_argument("--fps", type=float, default=30.0, help="target render rate")
    parser.add_argument("--point-size", type=float, default=2.0,
                         help="point size in pixels (smaller avoids overlapping points blending into a blob)")
    parser.add_argument("--bands", type=int, default=10,
                         help="number of discrete distance color bands (near=red, far=blue)")
    parser.add_argument("--invert-vertical", action="store_true",
                         help="flip the vertical axis if the scene renders upside down")
    parser.add_argument("--mirror-lr", action="store_true",
                         help="flip left/right if it still doesn't match reality after the built-in mirror fix")
    parser.add_argument("--axis-size", type=float, default=500.0,
                         help="length (mm) of the XYZ axis gizmo at the sensor's origin")
    args = parser.parse_args()

    print("Controls: left-drag to orbit, right-drag/scroll to zoom, middle-drag to pan.")
    print("Axis legend at the sensor's origin (0,0,0): "
          "RED = X (left/right), GREEN = depth (forward from sensor), BLUE = vertical"
          + (" [inverted]" if args.invert_vertical else "")
          + (" [mirrored]" if args.mirror_lr else "") + ".")
    print("If the scene looks upside down, restart with --invert-vertical.")
    print("If left/right still doesn't match reality, restart with --mirror-lr.")

    receiver = FrameReceiver(args.host, args.port)
    receiver.start()

    app = QtWidgets.QApplication(sys.argv)
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
    add_axis_label([a, 0, 0], "X: left/right", (255, 90, 90, 255))
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
        xyz, frame_cnt = receiver.get_latest()
        if xyz is None:
            return
        colors = make_banded_colors(xyz[:, 2], args.bands)  # color by native depth (sensor Z)
        plotted = remap_for_display(xyz, args.invert_vertical, args.mirror_lr)
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

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
