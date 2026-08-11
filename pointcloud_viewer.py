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
        print(f"Listening on {self.host}:{self.port} for the board's stream...")

        while not self.stop_event.is_set():
            conn, addr = srv.accept()
            print(f"Connected: {addr}")
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
            except ConnectionError:
                print("Board disconnected, waiting for a new connection...")
                self.connected_event.clear()
            finally:
                conn.close()


def make_colors(z: np.ndarray) -> np.ndarray:
    """Simple near=red / far=blue color gradient based on depth (Z)."""
    valid = np.isfinite(z) & (z > 0)
    if not np.any(valid):
        return np.tile([1.0, 1.0, 1.0, 1.0], (len(z), 1))
    zmin, zmax = z[valid].min(), z[valid].max()
    span = max(zmax - zmin, 1e-6)
    t = np.clip((z - zmin) / span, 0.0, 1.0)
    colors = np.zeros((len(z), 4), dtype=np.float32)
    colors[:, 0] = 1.0 - t  # red near
    colors[:, 2] = t  # blue far
    colors[:, 3] = 1.0
    colors[~valid] = 0.0  # hide invalid points (alpha 0)
    return colors


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="address to listen on")
    parser.add_argument("--port", type=int, default=5555, help="TCP port to listen on")
    parser.add_argument("--fps", type=float, default=30.0, help="target render rate")
    args = parser.parse_args()

    receiver = FrameReceiver(args.host, args.port)
    receiver.start()

    app = QtWidgets.QApplication(sys.argv)
    view = gl.GLViewWidget()
    view.setWindowTitle("HPS-3D160 Point Cloud")
    view.setCameraPosition(distance=2000)
    view.show()

    grid = gl.GLGridItem()
    grid.setSize(2000, 2000)
    grid.setSpacing(100, 100)
    view.addItem(grid)

    scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=3, pxMode=True)
    view.addItem(scatter)

    render_frame_count = 0
    render_window_start = QtCore.QElapsedTimer()
    render_window_start.start()

    def update():
        nonlocal render_frame_count
        xyz, frame_cnt = receiver.get_latest()
        if xyz is None:
            return
        colors = make_colors(xyz[:, 2])
        scatter.setData(pos=xyz, color=colors, size=3)

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
