#!/usr/bin/env python3
"""
Replay a point cloud recording saved by `pointcloud_viewer.py --record`.

Standalone in the sense that no sensor, board, or network connection is
needed -- it only needs the .npz file. The color-banding and axis-remapping
helpers are imported from pointcloud_viewer rather than copied, so replayed
frames are rendered with the exact same (hard-won) axis conventions as the
live view; see remap_for_display()'s docstring there for why that matters.

Frames are pulled out of the archive one at a time as they're displayed.
NpzFile decompresses lazily per member, so a multi-GB recording replays in
roughly one frame's worth of memory rather than being slurped up front.

Usage:
    python3 pointcloud_replay.py RECORDING.npz [--fps 30] [--point-size 2]
                                 [--bands 10] [--no-invert-vertical] [--no-mirror-lr]
                                 [--no-swap-xy] [--rotate 25]

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

from pointcloud_viewer import make_banded_colors, remap_for_display

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
    parser.add_argument("--fps", type=float, default=30.0, help="playback rate")
    parser.add_argument("--point-size", type=float, default=2.0,
                        help="point size in pixels (smaller avoids overlapping points blending into a blob)")
    parser.add_argument("--bands", type=int, default=10,
                        help="number of discrete distance color bands (near=red, far=violet)")
    parser.add_argument("--edge-filter-mm", type=float, default=300.0,
                        help="hide 'flying pixel' points whose depth jumps by more than this many "
                             "mm from an adjacent pixel in the sensor's scan grid; 0 disables")
    # Defaults below (--swap-xy, --mirror-lr, --invert-vertical, --rotate 25) match this
    # sensor's actual mounting, determined empirically by moving to a known physical side
    # of the sensor and watching where the point rendered. Use the --no-* form of the
    # boolean flags, or a different --rotate value, to override for a different mounting.
    parser.add_argument("--invert-vertical", action=argparse.BooleanOptionalAction, default=True,
                        help="flip the vertical axis if the scene renders upside down "
                             "(default: on; pass --no-invert-vertical to disable)")
    parser.add_argument("--mirror-lr", action=argparse.BooleanOptionalAction, default=True,
                        help="flip left/right if it doesn't match reality "
                             "(default: on; pass --no-mirror-lr to disable)")
    parser.add_argument("--swap-xy", action=argparse.BooleanOptionalAction, default=True,
                        help="swap the native X/Y axes if left/right motion shows up as vertical motion "
                             "on screen (or vice versa) -- indicates the sensor is mounted rolled 90 "
                             "degrees from what's assumed; re-check --invert-vertical/--mirror-lr after "
                             "(default: on; pass --no-swap-xy to disable)")
    parser.add_argument("--rotate", type=float, default=25.0, metavar="DEGREES",
                        help="yaw the scene by this many degrees around the vertical axis, e.g. if the "
                             "sensor was mounted facing a different direction than assumed (default: 25)")
    parser.add_argument("--invert-depth", action="store_true",
                        help="flip forward/backward if depth still comes out backwards after --rotate "
                             "(a rotation alone can't fix this -- see remap_for_display()'s docstring)")
    parser.add_argument("--axis-size", type=float, default=500.0,
                        help="length (mm) of the XYZ axis gizmo at the sensor's origin")
    parser.add_argument("--elevation", type=float, default=20.0,
                        help="initial camera elevation in degrees (0 = level/front-on, matches the "
                             "vertical axis exactly; the default 20 tilts down slightly)")
    parser.add_argument("--azimuth", type=float, default=-60.0,
                        help="initial camera azimuth in degrees around the vertical axis (0/-90/90/180 "
                             "give straight-on views; the default -60 is an oblique 3/4 view)")
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
    view.setCameraPosition(distance=2000, elevation=args.elevation, azimuth=args.azimuth)
    view.show()

    grid = gl.GLGridItem()
    grid.setSize(2000, 2000)
    grid.setSpacing(100, 100)
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

    status = gl.GLTextItem(pos=[0, 0, args.axis_size * 2.5], text="", color=(220, 220, 220, 255))
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
