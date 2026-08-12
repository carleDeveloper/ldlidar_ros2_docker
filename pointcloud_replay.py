#!/usr/bin/env python3
"""
Replay a point cloud recording saved by `pointcloud_viewer.py --record`.

Standalone in the sense that no sensor, board, or network connection is
needed -- it only needs the .npz file. The color-banding, axis-remapping,
and argument-definition helpers are imported from pointcloud_viewer rather
than copied, so replayed frames are rendered with the exact same (hard-won)
axis conventions -- and accept the same flags and defaults -- as the live
view; see remap_for_display()'s docstring there for why that matters.

Frames are pulled out of the archive one at a time as they're displayed.
NpzFile decompresses lazily per member, so a multi-GB recording replays in
roughly one frame's worth of memory rather than being slurped up front.

Usage:
    python3 pointcloud_replay.py RECORDING.npz [--fps 30] [--point-size 2]
                                 [--bands 10] [--no-invert-vertical] [--no-mirror-lr]
                                 [--no-swap-xy] [--rotate 25] [--grid-size 2000]
                                 [--grid-spacing 100] [--camera-distance 2000]

Note: --swap-xy, --mirror-lr, --invert-vertical default to on and --rotate
defaults to 25 degrees, matching this sensor's actual mounting as determined
empirically. Pass the --no-* form of a boolean flag, or a different --rotate
value, to override for a different mounting.

Controls:
    space        pause / resume
    left/right   step one frame (also pauses)
    home/end     jump to first / last frame
    mouse        left-drag orbit, right-drag or scroll zoom, middle-drag pan
"""
import argparse
import re
import signal
import sys
import zipfile

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets

from pointcloud_viewer import add_visualization_args, make_banded_colors, remap_for_display

FRAME_KEY_RE = re.compile(r"^frame_\d+$")


def load_recording(path: str):
    """Open the archive and return (npzfile, ordered frame keys, frame_info).

    Returns the NpzFile itself (not the frames) so members stay compressed on
    disk until the frame is actually displayed. `frame_info` is small enough
    to read eagerly, and may be absent if the recording was interrupted
    before the archive got finalized.
    """
    data = np.load(path)
    frame_keys = sorted(k for k in data.files if FRAME_KEY_RE.match(k))
    frame_info = data["frame_info"] if "frame_info" in data.files else None
    return data, frame_keys, frame_info


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", help="path to a .npz recording")
    add_visualization_args(parser, fps_help="playback rate")
    parser.add_argument("--status-height", type=float, default=None, metavar="MM",
                        help="height (mm) above the origin of the frame-counter label "
                             "(default: 2.5x --axis-size)")
    parser.add_argument("--loop", action="store_true",
                        help="restart at the end instead of holding on the last frame")
    args = parser.parse_args()

    try:
        data, frame_keys, frame_info = load_recording(args.recording)
    except (OSError, ValueError, zipfile.BadZipFile) as exc:
        # BadZipFile typically means the recorder was killed before it could
        # write the zip's central directory (i.e. SIGKILL rather than Ctrl+C),
        # which leaves the archive unreadable as a whole.
        sys.exit(f"Could not read {args.recording}: {exc}")

    if not frame_keys:
        sys.exit(f"{args.recording} contains no frames.")

    print(f"Loaded {len(frame_keys)} frames from {args.recording}")
    if frame_info is None:
        print("No frame_info member: the recording was probably cut short, "
              "so sensor frame counters are unavailable.")
    elif len(frame_info) != len(frame_keys):
        print(f"frame_info covers {len(frame_info)} of {len(frame_keys)} frames.")

    # Gaps mean the recorder dropped frames; worth knowing before concluding
    # that something in the scene moved discontinuously.
    if frame_info is not None and len(frame_info) > 1:
        deltas = np.diff(frame_info[:, 0].astype(np.int64))
        gaps = deltas[deltas > 1]
        if gaps.size:
            print(f"Note: {gaps.size} gap(s) in the sensor frame counter "
                  f"({int(gaps.sum() - gaps.size)} frames dropped while recording).")

    app = QtWidgets.QApplication(sys.argv)

    # Same rationale as pointcloud_viewer: Qt's event loop swallows a plain
    # KeyboardInterrupt raised inside a paint callback, so ask Qt to quit
    # instead, and keep a no-op timer running so signals get delivered at all.
    def request_quit(_signum, _frame):
        print("\nShutting down...")
        app.quit()

    signal.signal(signal.SIGINT, request_quit)
    signal.signal(signal.SIGTERM, request_quit)
    signal_timer = QtCore.QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)

    view = gl.GLViewWidget()
    view.setWindowTitle(f"Replay: {args.recording}")
    view.setCameraPosition(distance=args.camera_distance, elevation=args.elevation,
                           azimuth=args.azimuth)
    view.show()

    grid = gl.GLGridItem()
    grid.setSize(args.grid_size, args.grid_size)
    grid.setSpacing(args.grid_spacing, args.grid_spacing)
    view.addItem(grid)

    axis = gl.GLAxisItem()
    axis.setSize(args.axis_size, args.axis_size, args.axis_size)
    view.addItem(axis)

    a = args.axis_size * 1.05
    for pos, text, color in (
        ([a, 0, 0], "X: left/right", (255, 90, 90, 255)),
        ([0, a, 0], "depth (fwd)", (90, 255, 90, 255)),
        ([0, 0, a], "vertical (inverted)" if args.invert_vertical else "vertical",
         (90, 90, 255, 255)),
    ):
        view.addItem(gl.GLTextItem(pos=pos, text=text, color=color))

    scatter = gl.GLScatterPlotItem(pos=np.zeros((1, 3)), size=args.point_size, pxMode=True)
    scatter.setGLOptions("opaque")
    view.addItem(scatter)

    # Keyed off the axis gizmo by default so the label clears it at any --axis-size.
    status_height = args.status_height if args.status_height is not None else args.axis_size * 2.5
    status = gl.GLTextItem(pos=[0, 0, status_height], text="", color=(220, 220, 220, 255))
    view.addItem(status)

    state = {"index": 0, "paused": False}

    def show_frame():
        idx = state["index"]
        # Frames are stored as (height, width, 3) to preserve the sensor grid;
        # grab the grid dims (for the edge filter) before flattening to the
        # (N, 3) point list the render path expects.
        frame = data[frame_keys[idx]]
        height, width = frame.shape[:2] if frame.ndim == 3 else (None, None)
        xyz = frame.reshape(-1, 3)
        colors = make_banded_colors(xyz[:, 2], args.bands, width, height,
                                    args.edge_filter_mm)  # color by native depth (sensor Z)
        scatter.setData(pos=remap_for_display(xyz, args.invert_vertical, args.mirror_lr,
                                              args.rotate, args.invert_depth, args.swap_xy),
                        color=colors, size=args.point_size)

        label = f"frame {idx + 1}/{len(frame_keys)}"
        if frame_info is not None and idx < len(frame_info):
            label += f"  (sensor frame_cnt={frame_info[idx, 0]})"
        if state["paused"]:
            label += "  [paused]"
        status.setData(text=label)  # GLTextItem has no setText()

    def seek(idx: int):
        state["index"] = idx % len(frame_keys)
        state["paused"] = True  # stepping implies you want to stop and look
        show_frame()

    def advance():
        if state["paused"]:
            return
        if state["index"] >= len(frame_keys) - 1 and not args.loop:
            state["paused"] = True  # hold on the final frame instead of restarting
            show_frame()
            return
        state["index"] = (state["index"] + 1) % len(frame_keys)
        show_frame()

    timer = QtCore.QTimer()
    timer.timeout.connect(advance)
    timer.start(max(int(1000 / args.fps), 1))

    key = QtCore.Qt.Key

    def on_key(event):
        k = event.key()
        if k == key.Key_Space:
            state["paused"] = not state["paused"]
            show_frame()
        elif k == key.Key_Right:
            seek(state["index"] + 1)
        elif k == key.Key_Left:
            seek(state["index"] - 1)
        elif k == key.Key_Home:
            seek(0)
        elif k == key.Key_End:
            seek(len(frame_keys) - 1)
        else:
            gl.GLViewWidget.keyPressEvent(view, event)

    view.keyPressEvent = on_key

    print("Controls: space pause/resume, left/right step, home/end jump, "
          "left-drag orbit, right-drag/scroll zoom, middle-drag pan.")
    show_frame()

    exit_code = app.exec()
    data.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
