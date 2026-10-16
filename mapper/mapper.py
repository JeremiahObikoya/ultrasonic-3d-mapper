#!/usr/bin/env python3
"""
Live 3D mapper for the pan/tilt ultrasonic scanner.

    pip install pyserial matplotlib numpy scipy

    python mapper.py --port COM3            # Windows
    python mapper.py --port /dev/ttyUSB0    # Linux / macOS: /dev/cu.usbmodemXXXX
    python mapper.py --demo                 # no hardware: simulated room

Coordinate system (sensor at origin):
    X = right, Y = forward, Z = up      (all in cm)

Keys in the plot window:  d = clear the map
"""

import argparse
import csv
import math
import queue
import random
import threading
import time

import matplotlib.pyplot as plt
import matplotlib.cm as mcm
import matplotlib.colors as mcolors
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.spatial import Delaunay


# ----------------------------------------------------------------------
#  Arguments
# ----------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Live 3D ultrasonic map")
    p.add_argument("--port", help="serial port (auto-detected if omitted)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--demo", action="store_true", help="simulate a scanner, no hardware needed")
    p.add_argument("--max-range", type=float, default=200, help="plot half-size in cm")
    p.add_argument("--yaw-center", type=float, default=90, help="servo yaw angle that points straight ahead")
    p.add_argument("--pitch-center", type=float, default=90, help="servo pitch angle that is level")
    p.add_argument("--invert-yaw", action="store_true", help="flip left/right")
    p.add_argument("--invert-pitch", action="store_true", help="flip up/down")
    p.add_argument("--accumulate", action="store_true",
                   help="keep points from previous auto scans (default: new scan replaces old)")
    p.add_argument("--log", help="save all points to this CSV file on exit")
    return p.parse_args()


# ----------------------------------------------------------------------
#  Geometry
# ----------------------------------------------------------------------
def angles_to_direction(yaw, pitch, a):
    az = math.radians((yaw - a.yaw_center) * (-1 if a.invert_yaw else 1))
    el = math.radians((pitch - a.pitch_center) * (-1 if a.invert_pitch else 1))
    return (math.cos(el) * math.sin(az), math.cos(el) * math.cos(az), math.sin(el))


def to_xyz(yaw, pitch, dist, a):
    dx, dy, dz = angles_to_direction(yaw, pitch, a)
    return dist * dx, dist * dy, dist * dz


# ----------------------------------------------------------------------
#  Data sources (both push raw text lines into a queue)
# ----------------------------------------------------------------------
def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    if not ports:
        raise SystemExit("No serial ports found. Plug in the Arduino or use --port.")
    for p in ports:
        desc = f"{p.description} {p.manufacturer}".lower()
        if any(k in desc for k in ("arduino", "ch340", "usb serial", "usb-serial", "cp210")):
            return p.device
    return ports[0].device


def serial_source(port, baud, q, stop):
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        q.put(f"# ERROR opening {port}: {e}")
        return
    q.put(f"# Connected to {port}")
    time.sleep(2)               # Arduino resets when the port opens
    ser.reset_input_buffer()
    while not stop.is_set():
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except serial.SerialException as e:
            q.put(f"# Serial error: {e}")
            break
        if line:
            q.put(line)
    ser.close()


def fake_distance(yaw, pitch, a):
    """Ray-cast a box room with a ball in it."""
    d = angles_to_direction(yaw, pitch, a)
    candidates = []
    if d[0] > 1e-6:  candidates.append(150 / d[0])
    if d[0] < -1e-6: candidates.append(-150 / d[0])
    if d[1] > 1e-6:  candidates.append(250 / d[1])
    if d[2] > 1e-6:  candidates.append(190 / d[2])
    if d[2] < -1e-6: candidates.append(-60 / d[2])
    t = min(candidates) if candidates else 400
    c, r = (50, 150, -20), 40
    b = d[0] * c[0] + d[1] * c[1] + d[2] * c[2]
    disc = b * b - (c[0] ** 2 + c[1] ** 2 + c[2] ** 2 - r * r)
    if disc >= 0 and b - math.sqrt(disc) > 0:
        t = min(t, b - math.sqrt(disc))
    return min(t, 400)


def demo_source(q, stop, a):
    """Emits the same text protocol the Arduino does."""
    yaws = list(range(20, 161, 5))
    pitches = list(range(60, 121, 10))
    q.put("# DEMO mode (simulated data)")
    q.put("M,AUTO")
    while not stop.is_set():
        q.put("S,START")
        for i, p in enumerate(pitches):
            row = yaws if i % 2 == 0 else yaws[::-1]
            for y in row:
                if stop.is_set():
                    return
                d = fake_distance(y, p, a) + random.gauss(0, 0.8)
                q.put(f"D,{y:.1f},{p:.1f},{d:.1f},A")
                time.sleep(0.03)
        q.put("S,END")
        time.sleep(2)


# ----------------------------------------------------------------------
#  Map state
# ----------------------------------------------------------------------
class MapState:
    def __init__(self, args):
        self.args = args
        self.points = {}          # (yaw, pitch) -> (x, y, z, dist); newest reading wins
        self.mode = "?"
        self.scans_done = 0
        self.dirty = True

    def handle_line(self, line):
        a = self.args
        if line.startswith("#"):
            print(line)
            return
        parts = line.split(",")
        try:
            if parts[0] == "D" and len(parts) >= 5:
                yaw, pitch, dist = float(parts[1]), float(parts[2]), float(parts[3])
                if dist <= 0:     # no valid echo at this point
                    return
                self.points[(round(yaw, 1), round(pitch, 1))] = (*to_xyz(yaw, pitch, dist, a), dist)
                self.dirty = True
            elif parts[0] == "S":
                if parts[1] == "START" and not a.accumulate:
                    self.points.clear()
                    self.dirty = True
                elif parts[1] == "END":
                    self.scans_done += 1
                    print(f"Scan {self.scans_done} complete ({len(self.points)} points)")
            elif parts[0] == "M":
                self.mode = parts[1]
                print(f"Mode: {self.mode}")
            else:
                print(line)
        except (ValueError, IndexError):
            pass                  # ignore corrupted lines

    def clear(self):
        self.points.clear()
        self.dirty = True

    def save_csv(self, path):
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x_cm", "y_cm", "z_cm", "distance_cm"])
            for x, y, z, d in self.points.values():
                w.writerow([f"{x:.2f}", f"{y:.2f}", f"{z:.2f}", f"{d:.2f}"])
        print(f"Saved {len(self.points)} points to {path}")


# ----------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------
def main():
    args = parse_args()
    q = queue.Queue()
    stop = threading.Event()
    state = MapState(args)

    if args.demo:
        src = threading.Thread(target=demo_source, args=(q, stop, args), daemon=True)
    else:
        port = args.port or find_port()
        print(f"Using port {port}")
        src = threading.Thread(target=serial_source, args=(port, args.baud, q, stop), daemon=True)
    src.start()

    R = args.max_range
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(-R, R); ax.set_ylim(-R, R); ax.set_zlim(-R, R)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X right (cm)"); ax.set_ylabel("Y forward (cm)"); ax.set_zlabel("Z up (cm)")
    ax.view_init(elev=20, azim=-90)
    ax.scatter([0], [0], [0], c="red", s=80, marker="^", label="sensor")

    # Colourmap / normalisation shared between surface and fallback scatter
    cmap = mcm.get_cmap("turbo_r")
    norm = mcolors.Normalize(vmin=0, vmax=R * 1.5)

    # Thin fallback scatter (visible only while < 3 points)
    scat = ax.scatter([0], [0], [0], c=[0], cmap=cmap, norm=norm, s=12, alpha=0.4)
    fig.colorbar(mcm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, shrink=0.6, label="distance (cm)")

    # Handle to the current surface so we can remove it each frame
    surf_handle = [None]

    def update(_frame):
        while True:
            try:
                state.handle_line(q.get_nowait())
            except queue.Empty:
                break
        if state.dirty and state.points:
            keys = list(state.points.keys())
            vals = np.array([state.points[k] for k in keys])
            xs, ys, zs, ds = vals[:, 0], vals[:, 1], vals[:, 2], vals[:, 3]

            # Remove the previous surface
            if surf_handle[0] is not None:
                surf_handle[0].remove()
                surf_handle[0] = None

            if len(keys) >= 3:
                # Triangulate in angle space for a clean mesh
                angles = np.array(keys, dtype=float)
                try:
                    tri = Delaunay(angles)
                    surf_handle[0] = ax.plot_trisurf(
                        xs, ys, zs,
                        triangles=tri.simplices,
                        cmap=cmap, norm=norm,
                        edgecolor="none", alpha=0.85, shade=True,
                    )
                    # Colour each face by mean distance of its vertices
                    face_dists = ds[tri.simplices].mean(axis=1)
                    face_colours = cmap(norm(face_dists))
                    surf_handle[0].set_facecolors(face_colours)
                    # Hide the scatter dots when we have a surface
                    scat._offsets3d = (np.array([]), np.array([]), np.array([]))
                    scat.set_array(np.array([]))
                except Exception:
                    # Degenerate point set → fall back to scatter
                    scat._offsets3d = (xs, ys, zs)
                    scat.set_array(ds)
            else:
                scat._offsets3d = (xs, ys, zs)
                scat.set_array(ds)

            state.dirty = False
        ax.set_title(f"Mode: {state.mode}   Points: {len(state.points)}   Scans: {state.scans_done}")
        return (scat,)

    def on_key(event):
        if event.key == "d":
            state.clear()
            if surf_handle[0] is not None:
                surf_handle[0].remove()
                surf_handle[0] = None

    fig.canvas.mpl_connect("key_press_event", on_key)
    ani = FuncAnimation(fig, update, interval=150, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        stop.set()
        if args.log:
            state.save_csv(args.log)


if __name__ == "__main__":
    main()
