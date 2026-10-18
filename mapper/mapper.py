#!/usr/bin/env python3
"""
Live 3D point cloud mapper for the pan/tilt ultrasonic scanner.

    pip install pyserial matplotlib numpy

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

import matplotlib
import matplotlib.cm as mcm
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


# ----------------------------------------------------------------------
#  Arguments
# ----------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Live 3D ultrasonic point cloud map")
    p.add_argument("--port", help="serial port (auto-detected if omitted)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--demo", action="store_true", help="simulate a scanner, no hardware needed")
    p.add_argument("--max-range", type=float, default=400.0, help="maximum sensor range in cm (default: 400.0)")
    p.add_argument("--floor-z", type=float, default=-60.0, help="floor plane elevation in cm (default: -60.0)")
    p.add_argument("--z-min", type=float, default=None, help="minimum Z limit in cm (default: floor_z - 20)")
    p.add_argument("--z-max", type=float, default=None, help="maximum Z limit in cm (default: max_range + 20)")
    p.add_argument("--yaw-center", type=float, default=90, help="servo yaw angle that points straight ahead")
    p.add_argument("--pitch-center", type=float, default=90, help="servo pitch angle that is level")
    p.add_argument("--invert-yaw", action="store_true", help="flip left/right")
    p.add_argument("--invert-pitch", dest="invert_pitch", action="store_true", default=True,
                   help="pitch < 90 points UP (default: True)")
    p.add_argument("--no-invert-pitch", dest="invert_pitch", action="store_false",
                   help="pitch > 90 points UP")
    p.add_argument("--clear-on-start", action="store_true",
                   help="wipe map when each new scan starts (default: False, points update in-place)")
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
    """Emits the same text protocol the Arduino does with bidirectional sweep."""
    yaws = list(range(20, 161, 5))
    pitches_down = list(range(60, 121, 10))
    pitches_up = pitches_down[-2:0:-1]  # intermediate rows on the way back up
    q.put("# DEMO mode (simulated data)")
    q.put("M,AUTO")
    q.put("S,START")

    yaw_reversed = False
    sweep_sequence = [pitches_down, pitches_up]
    seq_idx = 0

    while not stop.is_set():
        current_pitches = sweep_sequence[seq_idx % 2]
        for p in current_pitches:
            row = yaws[::-1] if yaw_reversed else yaws
            for y in row:
                if stop.is_set():
                    return
                d = fake_distance(y, p, a) + random.gauss(0, 0.8)
                q.put(f"D,{y:.1f},{p:.1f},{d:.1f},A")
                time.sleep(0.02)
            yaw_reversed = not yaw_reversed
        q.put("S,END")
        q.put("S,START")
        seq_idx += 1


# ----------------------------------------------------------------------
#  Map state
# ----------------------------------------------------------------------
class MapState:
    def __init__(self, args):
        self.args = args
        self.points = {}          # (yaw, pitch) -> (x, y, z, dist); newest reading wins
        self.last_reading = None  # (x, y, z, dist, yaw, pitch) of latest measurement
        self.last_raw = None      # (yaw, pitch, dist)
        self.mode = "STANDBY"
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
                xyz = to_xyz(yaw, pitch, dist, a)
                self.points[(round(yaw, 1), round(pitch, 1))] = (*xyz, dist)
                self.last_reading = (*xyz, dist, yaw, pitch)
                self.last_raw = (yaw, pitch, dist)
                self.dirty = True
            elif parts[0] == "S":
                if parts[1] == "START":
                    if a.clear_on_start:
                        self.points.clear()
                        self.last_reading = None
                        self.last_raw = None
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

    def toggle_invert_pitch(self):
        self.args.invert_pitch = not self.args.invert_pitch
        self.recompute_all()

    def toggle_invert_yaw(self):
        self.args.invert_yaw = not self.args.invert_yaw
        self.recompute_all()

    def recompute_all(self):
        new_points = {}
        for (yaw, pitch), val in self.points.items():
            dist = val[3]
            xyz = to_xyz(yaw, pitch, dist, self.args)
            new_points[(yaw, pitch)] = (*xyz, dist)
        self.points = new_points
        if self.last_raw:
            lyaw, lpitch, ld = self.last_raw
            self.last_reading = (*to_xyz(lyaw, lpitch, ld, self.args), ld, lyaw, lpitch)
        self.dirty = True

    def clear(self):
        self.points.clear()
        self.last_reading = None
        self.last_raw = None
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
    z_floor = args.floor_z
    x_min, x_max = -R, R
    y_min, y_max = -20.0, R + 20.0
    z_min = args.z_min if args.z_min is not None else (z_floor - 20.0)
    z_max = args.z_max if args.z_max is not None else (R + 20.0)

    # 1. Figure & 3D Axes with Dark Robotics / LiDAR Styling
    fig = plt.figure(figsize=(10, 8), facecolor="#0b0f19")
    if hasattr(fig.canvas.manager, "set_window_title"):
        fig.canvas.manager.set_window_title("3D Ultrasonic Scanner — Live Point Cloud")
    ax = fig.add_subplot(111, projection="3d", facecolor="#0b0f19")
    ax.set_facecolor("#0b0f19")

    # Dark background panes
    pane_color = (0.05, 0.08, 0.14, 0.9)
    ax.xaxis.set_pane_color(pane_color)
    ax.yaxis.set_pane_color(pane_color)
    ax.zaxis.set_pane_color(pane_color)

    # Clean grid lines & tick parameters
    ax.grid(True, color="#1e293b", linestyle="--", linewidth=0.6, alpha=0.8)
    ax.tick_params(colors="#94a3b8", labelsize=8)
    ax.set_xlabel("X (Right) cm", color="#cbd5e1", labelpad=8, fontsize=9)
    ax.set_ylabel("Y (Forward) cm", color="#cbd5e1", labelpad=8, fontsize=9)
    ax.set_zlabel("Z (Up) cm", color="#cbd5e1", labelpad=8, fontsize=9)

    # 2. View bounds & True Physical 1:1:1 Aspect Ratio
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    ax.set_box_aspect(((x_max - x_min), (y_max - y_min), (z_max - z_min)))
    ax.view_init(elev=22, azim=-68)

    # 3. Ground Radar Reference Grid (at z_floor)
    arc_angles = np.linspace(-np.radians(75), np.radians(75), 60)
    for r in range(50, int(R) + 50, 50):
        if r <= y_max:
            xs = r * np.sin(arc_angles)
            ys = r * np.cos(arc_angles)
            zs = np.full_like(xs, z_floor)
            ax.plot(xs, ys, zs, color="#1e3a5f", linestyle=":", linewidth=0.8, alpha=0.65)
            ax.text(0, r, z_floor, f"{r} cm", color="#475569", fontsize=7.5,
                    ha="center", va="bottom", zorder=2)

    for deg in [-60, -30, 0, 30, 60]:
        rad = np.radians(deg)
        r_spoke = min(y_max, R)
        ax.plot([0, r_spoke * np.sin(rad)], [0, r_spoke * np.cos(rad)], [z_floor, z_floor],
                color="#1e293b", linestyle="--", linewidth=0.7, alpha=0.6)

    # 4. Scanner Station (Pedestal + Base + Sensor Head)
    ax.plot([0, 0], [0, 0], [z_floor, 0], color="#0ea5e9", linewidth=2.2, alpha=0.75, label="Scanner Mount")
    base_angles = np.linspace(0, 2 * np.pi, 24)
    ax.plot(8 * np.cos(base_angles), 8 * np.sin(base_angles), np.full_like(base_angles, z_floor),
            color="#0ea5e9", linewidth=1.2, alpha=0.6)
    ax.scatter([0], [0], [0], c="#38bdf8", s=110, marker="^", depthshade=False, label="Sensor Origin")
    ax.plot([0, 0], [0, 15], [0, 0], color="#38bdf8", linewidth=2.0, alpha=0.9)

    # 5. Live Acoustic Pulse Beam & Active Ping Marker
    beam_line, = ax.plot([], [], [], color="#38bdf8", linewidth=1.5, alpha=0.85, linestyle="-")
    ping_dot = ax.scatter([], [], [], c="#f43f5e", s=80, marker="o", depthshade=False)

    # 6. Point Cloud Scatter (Dual-layer emissive LiDAR styling)
    cmap = matplotlib.colormaps["turbo_r"]
    norm = mcolors.Normalize(vmin=0, vmax=R)
    scat_glow = ax.scatter([], [], [], c=[], cmap=cmap, norm=norm, s=55, alpha=0.25, edgecolors="none")
    scat_core = ax.scatter([], [], [], c=[], cmap=cmap, norm=norm, s=18, alpha=0.92, edgecolors="#ffffff", linewidths=0.2)

    cbar = fig.colorbar(mcm.ScalarMappable(norm=norm, cmap=cmap), ax=ax, shrink=0.55, aspect=18, pad=0.08)
    cbar.set_label("Echo Distance (cm)", color="#94a3b8", fontsize=9, labelpad=8)
    cbar.ax.tick_params(colors="#94a3b8", labelsize=8)
    cbar.outline.set_edgecolor("#1e293b")

    # 7. Telemetry HUD Banner & Footer
    hud_banner = ax.set_title(
        "3D ULTRASONIC SCANNER  |  INITIALIZING...",
        color="#e2e8f0",
        fontsize=10,
        fontweight="bold",
        pad=14,
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#0f172a", edgecolor="#1e293b", alpha=0.9)
    )
    fig.text(0.5, 0.02, "[D] Clear Map   •   [P] Flip Pitch (Up/Down)   •   [Y] Flip Yaw   •   Rotate/Zoom: Mouse",
             ha="center", color="#64748b", fontsize=8.5)

    def update(_frame):
        while True:
            try:
                state.handle_line(q.get_nowait())
            except queue.Empty:
                break
        if state.dirty:
            if state.points:
                pts = np.array(list(state.points.values()))
                xs, ys, zs, ds = pts[:, 0], pts[:, 1], pts[:, 2], pts[:, 3]
                scat_glow._offsets3d = (xs, ys, zs)
                scat_glow.set_array(ds)
                scat_core._offsets3d = (xs, ys, zs)
                scat_core.set_array(ds)
            else:
                empty = np.array([])
                scat_glow._offsets3d = (empty, empty, empty)
                scat_glow.set_array(empty)
                scat_core._offsets3d = (empty, empty, empty)
                scat_core.set_array(empty)

            if state.last_reading:
                lx, ly, lz, ld, lyaw, lpitch = state.last_reading
                beam_line.set_data_3d([0, lx], [0, ly], [0, lz])
                ping_dot._offsets3d = ([lx], [ly], [lz])
                hud_ping = f"YAW: {lyaw:.1f}°  PITCH: {lpitch:.1f}°  DIST: {ld:.1f} cm"
            else:
                beam_line.set_data_3d([], [], [])
                ping_dot._offsets3d = ([], [], [])
                hud_ping = "STANDBY"

            mode_str = "AUTO SCAN" if "AUTO" in state.mode else ("JOYSTICK" if "JOY" in state.mode else state.mode)
            pitch_stat = "INV" if state.args.invert_pitch else "NORM"
            yaw_stat = "INV" if state.args.invert_yaw else "NORM"
            hud_banner.set_text(
                f"MODE: {mode_str}   |   PTS: {len(state.points)}   |   SCANS: {state.scans_done}   |   {hud_ping}   |   PITCH:{pitch_stat} YAW:{yaw_stat}"
            )
            state.dirty = False

        return (scat_core, scat_glow, beam_line, ping_dot)

    def on_key(event):
        if event.key == "d":
            state.clear()
        elif event.key == "p":
            state.toggle_invert_pitch()
            print(f"[HOTKEY] Pitch inverted: {state.args.invert_pitch} (all points flipped)")
        elif event.key == "y":
            state.toggle_invert_yaw()
            print(f"[HOTKEY] Yaw inverted: {state.args.invert_yaw} (all points flipped)")

    fig.canvas.mpl_connect("key_press_event", on_key)
    ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        stop.set()
        if args.log:
            state.save_csv(args.log)


if __name__ == "__main__":
    main()
