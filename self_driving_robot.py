#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import math
import struct
import threading
import heapq
import numpy as np
import argparse
import signal

# Configure matplotlib / Qt platform depending on display environment.
# - If no X display is present, use Agg (non-interactive).
# - If running under Wayland, avoid QtWayland's QWindow::requestActivate warnings by
#   forcing an XCB platform for Qt (falls back to non-Qt backend if necessary).
if not os.environ.get("DISPLAY"):
    import matplotlib
    matplotlib.use("Agg")
else:
    # Detect Wayland session (common env var XDG_SESSION_TYPE=wayland and/or WAYLAND_DISPLAY)
    xdg_type = os.environ.get("XDG_SESSION_TYPE", "").lower()
    wayland_present = bool(os.environ.get("WAYLAND_DISPLAY")) or xdg_type == "wayland"
    if wayland_present:
        # Force Qt to use XCB (X11) platform plugin to avoid Wayland requestActivate warnings
        # This must be set before importing any Qt modules / matplotlib backends that use Qt.
        # If XCB is not available on the system, you can alternatively force a different
        # matplotlib backend (e.g. TkAgg) or set QT_QPA_PLATFORM=offscreen for headless use.
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import matplotlib.pyplot as plt
import can
from rplidar import RPLidar

# =========================
# Robot and system params
# =========================
WHEEL_RADIUS = 0.04     # meters (80 mm diameter)
BASELINE     = 0.30     # meters between wheel centers

MAX_V        = 0.45     # m/s
MAX_W        = 2.0      # rad/s
VEL_RAMP     = 0.06     # m/s per cycle
W_RAMP       = 0.12     # rad/s per cycle

GRID_RES     = 0.07
GRID_SIZE    = 200
GRID_HALF    = GRID_SIZE // 2

MIN_RANGE    = 0.15
MAX_RANGE    = 6.0

CAN_INTERFACE = 'can0'
BITRATE       = 250000
NODE_IDS      = [1, 2]

AXIS_STATE_IDLE                = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# =========================
# CAN helpers
# =========================
def init_can():
    # Configure SocketCAN interface
    os.system(f"sudo ip link set {CAN_INTERFACE} down")
    os.system(f"sudo ip link set {CAN_INTERFACE} type can bitrate {BITRATE}")
    os.system(f"sudo ip link set {CAN_INTERFACE} up")
    time.sleep(0.3)
    # Use 'interface' instead of deprecated 'bustype'
    return can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')

def shutdown_can(bus):
    try:
        bus.shutdown()
    except Exception:
        pass

def set_axis_state(bus, nid, state):
    arb_id = (nid << 5) | 0x07
    data   = struct.pack('<I', state)
    msg    = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def set_velocity(bus, nid, vel_turns_per_s, torque_ff=0.0):
    arb_id = (nid << 5) | 0x0D
    data   = struct.pack('<ff', float(vel_turns_per_s), float(torque_ff))
    msg    = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

# =========================
# Odometry
# =========================
class Odom:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

    def update(self, v_left_turns_s, v_right_turns_s, dt):
        vL = 2.0 * math.pi * WHEEL_RADIUS * v_left_turns_s
        vR = 2.0 * math.pi * WHEEL_RADIUS * v_right_turns_s
        v  = 0.5 * (vL + vR)
        w  = (vR - vL) / BASELINE
        self.x  += v * math.cos(self.th) * dt
        self.y  += v * math.sin(self.th) * dt
        self.th += w * dt
        # Normalize angle to [-pi, pi]
        self.th = (self.th + math.pi) % (2*math.pi) - math.pi
        return v, w

# =========================
# Occupancy grid
# =========================
class Grid:
    def __init__(self):
        self.res = GRID_RES
        self.size = GRID_SIZE
        # discrete occupancy: -1 free, 0 unknown, 1 occupied
        self.grid = np.zeros((self.size, self.size), dtype=np.int8)
        # log-odds representation for probabilistic mapping
        self.logodds = np.zeros((self.size, self.size), dtype=np.float32)
        # thresholds and update constants (tunable)
        self.occ_threshold = 2.0
        self.free_threshold = -2.0
        self.hit_update = 1.0
        self.miss_update = -0.4

    def world_to_cell(self, x, y):
        half = self.size // 2
        cx = int(x / self.res) + half
        cy = int(y / self.res) + half
        return cx, cy

    def cell_to_world(self, cx, cy):
        half = self.size // 2
        x = (cx - half) * self.res
        y = (cy - half) * self.res
        return x, y

    def bresenham(self, x0, y0, x1, y1):
        x0, y0 = int(x0), int(y0)
        x1, y1 = int(x1), int(y1)
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            yield x0, y0
            if x0 == x1 and y0 == y1:
                break
            e2 = 2*err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def update_ray(self, rx, ry, rtheta, rng):
        """Update occupancy using a simple log-odds inverse sensor model.

        - cells along the ray (except the endpoint) get a miss update (less likely occupied)
        - the endpoint cell (if in range) gets a hit update (more likely occupied)
        The discrete occupancy grid `self.grid` is updated from `self.logodds` using thresholds.
        """
        ex = rx + rng * math.cos(rtheta)
        ey = ry + rng * math.sin(rtheta)
        sx, sy = self.world_to_cell(rx, ry)
        gx, gy = self.world_to_cell(ex, ey)
        # iterate through cells on the ray; treat final cell as hit
        cells = list(self.bresenham(sx, sy, gx, gy))
        if not cells:
            return
        # all but last are misses
        for cx, cy in cells[:-1]:
            if 0 <= cx < self.size and 0 <= cy < self.size:
                self.logodds[cx, cy] += self.miss_update
        # endpoint is a hit (if within grid)
        cx, cy = cells[-1]
        if 0 <= cx < self.size and 0 <= cy < self.size:
            self.logodds[cx, cy] += self.hit_update

        # clamp logodds to avoid runaway values
        np.clip(self.logodds, -10.0, 10.0, out=self.logodds)

        # update discrete grid from logodds thresholds
        self.grid.fill(0)
        self.grid[self.logodds > self.occ_threshold] = 1
        self.grid[self.logodds < self.free_threshold] = -1

    def save(self, path):
        """Save grid to a compressed npz file at `path`.

        Writes fields: grid, res, size. Caller may pass filename with or without .npz
        """
        if not path.endswith('.npz'):
            path = path + '.npz'
        # Save both discrete grid and logodds for future incremental mapping
        np.savez_compressed(path,
                            grid=self.grid,
                            logodds=self.logodds,
                            res=self.res,
                            size=self.size,
                            occ_threshold=self.occ_threshold,
                            free_threshold=self.free_threshold)

    @classmethod
    def load(cls, path):
        """Load grid from a .npz file and return a Grid instance."""
        if not path.endswith('.npz'):
            path = path + '.npz'
        data = np.load(path)
        g = cls()
        try:
            if 'logodds' in data:
                g.logodds = data['logodds']
                # derive discrete grid from loaded logodds
                g.grid.fill(0)
                g.grid[g.logodds > g.occ_threshold] = 1
                g.grid[g.logodds < g.free_threshold] = -1
            elif 'grid' in data:
                g.grid = data['grid']
                # keep logodds at zeros
            if 'res' in data:
                g.res = float(data['res'])
            if 'size' in data:
                g.size = int(data['size'])
                # if size differs, ensure arrays match
                if g.logodds.shape != (g.size, g.size):
                    g.logodds = np.zeros((g.size, g.size), dtype=np.float32)
                    if g.grid.shape == (g.size, g.size):
                        # reconstruct simple logodds from grid
                        g.logodds[g.grid == 1] = g.occ_threshold + 0.5
                        g.logodds[g.grid == -1] = g.free_threshold - 0.5
            # thresholds optional
            if 'occ_threshold' in data:
                g.occ_threshold = float(data['occ_threshold'])
            if 'free_threshold' in data:
                g.free_threshold = float(data['free_threshold'])
        except Exception as e:
            raise RuntimeError(f"Failed to load grid from {path}: {e}")
        return g

    def inflate(self, radius_m):
        """Inflate occupied cells by radius (meters) to create a safety margin for planning.

        This uses a circular neighborhood (based on euclidean distance) to mark
        nearby cells occupied. Implemented without external dependencies so it
        works offline on small maps.
        """
        if radius_m <= 0:
            return
        r_cells = max(1, int(math.ceil(radius_m / self.res)))
        occ = np.argwhere(self.grid == 1)
        if occ.size == 0:
            return
        new_grid = np.array(self.grid, copy=True)
        # precompute offsets within circular radius
        offs = []
        for dx in range(-r_cells, r_cells+1):
            for dy in range(-r_cells, r_cells+1):
                if dx*dx + dy*dy <= r_cells*r_cells:
                    offs.append((dx, dy))
        for cx, cy in occ:
            for dx, dy in offs:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.size and 0 <= ny < self.size:
                    new_grid[nx, ny] = 1
        self.grid = new_grid

    def export_ros_map(self, pgm_path, yaml_path, origin_x=0.0, origin_y=0.0):
        """Export grid as ROS map format (PGM image + YAML metadata).

        - pgm_path: path to write the PGM image
        - yaml_path: path to write the YAML metadata
        - origin_x, origin_y: origin offset in meters (bottom-left of map)
        """
        # Import here to avoid hard dependency
        try:
            from PIL import Image
        except ImportError:
            raise ImportError("PIL/Pillow required for PGM export. Install with: pip install Pillow")

        # Convert grid to ROS format: 0=unknown, 100=occupied, 254=free (in [0, 255])
        ros_grid = np.full(self.grid.shape, 127, dtype=np.uint8)  # unknown
        ros_grid[self.grid == 1] = 0    # occupied -> black
        ros_grid[self.grid == -1] = 254  # free -> white

        # Save PGM (transpose because ROS uses y-down, we use y-up)
        img = Image.fromarray(ros_grid.T, mode='L')
        img.save(pgm_path)

        # Write YAML metadata
        yaml_content = f"""image: {os.path.basename(pgm_path)}
resolution: {self.res}
origin: [{origin_x}, {origin_y}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
"""
        with open(yaml_path, 'w') as f:
            f.write(yaml_content)

# =========================
# LIDAR helpers
# =========================
def find_lidar_port():
    # Prefer typical USB serial devices first
    candidates = [f"/dev/ttyUSB{i}" for i in range(0, 4)]
    # Try ACM devices and fall back to onboard UARTs if needed
    candidates += [f"/dev/ttyACM{i}" for i in range(0, 2)]
    candidates += ["/dev/ttyAMA0", "/dev/ttyS0"]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None

# Global stop event set by signal handlers (used to shut down cleanly on Ctrl+C)
STOP_EVENT = threading.Event()


def _signal_handler(signum, frame):
    # Print a short message and set the global stop event.
    print(f"Received signal {signum}; initiating shutdown...")
    STOP_EVENT.set()

# =========================
# Autosave thread
# =========================
class AutosaveThread(threading.Thread):
    """Periodically saves grid to a temporary file and atomically swaps it to the target."""
    def __init__(self, grid, save_path, interval_s=10.0):
        super().__init__(daemon=True)
        self.grid = grid
        self.save_path = save_path
        self.interval_s = interval_s
        self.running = True

    def run(self):
        while self.running and not STOP_EVENT.is_set():
            try:
                time.sleep(self.interval_s)
                if STOP_EVENT.is_set():
                    break
                # Save to temp file, then atomically rename
                tmp_path = self.save_path + ".tmp"
                self.grid.save(tmp_path)
                # atomic rename (on POSIX systems)
                if os.path.exists(self.save_path):
                    os.remove(self.save_path)
                os.rename(tmp_path, self.save_path)
                # print(f"[Autosave] Saved to {self.save_path}")
            except Exception as e:
                print(f"[Autosave] Error: {e}")

    def stop(self):
        self.running = False

# =========================
# Lidar thread
# =========================
class LidarThread(threading.Thread):
    def __init__(self, port=None, baud_list=None, max_retries=3):
        super().__init__(daemon=True)
        if port is None:
            port = find_lidar_port()
            if port is None:
                raise RuntimeError("No LIDAR serial port found. Plug in the device or set port='/dev/ttyUSBX'.")
        self.port = port
        # Common baudrates to try; many RPLidar A1/A2 use 115200 but some variants differ
        self.baud_list = baud_list or [115200, 256000, 230400, 128000, 57600]
        self.max_retries = max_retries
        self.lidar = None
        self.lock = threading.Lock()
        self.scan = []
        self.running = True

    def run(self):
        # Try to open RPLidar with multiple baudrates and a few retries
        opened = False
        last_exc = None
        for attempt in range(self.max_retries):
            for baud in self.baud_list:
                try:
                    print(f"Lidar: attempting open {self.port} @ {baud} (attempt {attempt+1})")
                    self.lidar = RPLidar(self.port, baudrate=baud, timeout=3)
                    # quick health check
                    try:
                        h = self.lidar.get_health()
                        print("Lidar health:", h)
                    except Exception as eh:
                        print(f"Lidar get_health failed: {eh}")
                    opened = True
                    break
                except Exception as e:
                    last_exc = e
                    print(f"Open failed for baud {baud}: {e}")
            if opened:
                break
            time.sleep(0.5)

        if not opened:
            print(f"Lidar thread: unable to open device {self.port} after {self.max_retries} retries: {last_exc}")
            return

        try:
            self.lidar.start_motor()
            for scan in self.lidar.iter_scans():
                if not self.running or STOP_EVENT.is_set():
                    break
                processed = []
                for (_, ang_deg, dist_mm) in scan:
                    rng_m = dist_mm / 1000.0
                    if rng_m < MIN_RANGE or rng_m > MAX_RANGE:
                        continue
                    processed.append((math.radians(ang_deg), rng_m))
                with self.lock:
                    self.scan = processed
        except Exception as e:
            # If we get structured errors like 'Wrong body size', produce extra debug info
            print(f"Lidar thread error: {e}")
            try:
                # Try to capture a small raw sample from the serial device for diagnosis
                try:
                    import serial
                    with serial.Serial(self.port, timeout=1) as s:
                        raw = s.read(64)
                        if raw:
                            print("Raw sample (hex):", raw.hex())
                        else:
                            print("No raw bytes available to sample.")
                except Exception as re:
                    print(f"Could not read raw bytes for debug: {re}")
            except Exception:
                pass
        finally:
            try:
                self.lidar.stop()
                self.lidar.disconnect()
            except Exception:
                pass

    def get_scan(self):
        with self.lock:
            return list(self.scan)

    def stop(self):
        self.running = False

# =========================
# Reactive planner
# =========================
def reactive_plan(scan, min_clear=0.10, max_v=MAX_V):
    if not scan:
        return 0.0, 0.0
    front = [(ang, rng) for (ang, rng) in scan
             if (ang <= math.radians(60)) or (ang >= math.radians(300))]
    if not front:
        return 0.2, 0.0
    _, min_rng = min(front, key=lambda s: s[1])
    v = max(0.0, min(max_v, (min_rng - min_clear)))
    # Steer away from nearest obstacle direction
    min_ang = min(front, key=lambda s: s[1])[0]
    a = ((min_ang + math.pi) % (2 * math.pi)) - math.pi
    w = -1.2 * a
    return v, w

# =========================
# A* planner
# =========================
def astar(grid, start, goal):
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    if not (0 <= start[0] < grid.size and 0 <= start[1] < grid.size):
        return None
    if not (0 <= goal[0] < grid.size and 0 <= goal[1] < grid.size):
        return None
    if grid.grid[start[0], start[1]] == 1 or grid.grid[goal[0], goal[1]] == 1:
        return None

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    nbrs = [(1,0),(-1,0),(0,1),(0,-1)]

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        cx, cy = current
        for dx, dy in nbrs:
            nx, ny = cx+dx, cy+dy
            if not (0 <= nx < grid.size and 0 <= ny < grid.size):
                continue
            if grid.grid[nx, ny] == 1:
                continue
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get((nx, ny), float('inf')):
                came_from[(nx, ny)] = current
                g_score[(nx, ny)] = tentative_g
                f_score = tentative_g + heuristic((nx, ny), goal)
                heapq.heappush(open_set, (f_score, (nx, ny)))
    return None


# =========================
# Pure pursuit controller
# =========================
def pure_pursuit(path_world, odom, lookahead=0.35, max_v=MAX_V):
    """Simple pure-pursuit style controller returning (v, w).

    - path_world: list of (x,y) waypoints in world coordinates
    - odom: Odom instance with x,y,th
    Looks for the first path point at distance >= lookahead and steers towards it.
    """
    if not path_world:
        return 0.0, 0.0
    # find first point ahead at >= lookahead
    target = None
    for px, py in path_world:
        dx = px - odom.x
        dy = py - odom.y
        d = math.hypot(dx, dy)
        if d >= lookahead:
            target = (px, py, d)
            break
    if target is None:
        # fallback to last point
        px, py = path_world[-1]
        dx = px - odom.x
        dy = py - odom.y
        d = math.hypot(dx, dy)
        target = (px, py, d)

    px, py, d = target
    angle_to_target = math.atan2(py - odom.y, px - odom.x)
    angle_diff = ((angle_to_target - odom.th + math.pi) % (2 * math.pi)) - math.pi

    # Steering law: curvature kappa = 2*sin(angle_diff)/L where L = lookahead
    L = max(0.01, lookahead)
    kappa = 2.0 * math.sin(angle_diff) / L
    # Convert curvature to angular velocity w = v * kappa
    # Choose v reduced by heading error
    v = max_v * max(0.0, math.cos(angle_diff))
    # if target is very close, slow down
    if d < 0.2:
        v = min(v, 0.2)
    w = v * kappa
    # clamp
    v = max(-max_v, min(max_v, v))
    w = max(-MAX_W, min(MAX_W, w))
    return v, w

# =========================
# Visualization with arrow
# =========================
def visualize(grid, odom, path=None, goal=None):
    plt.clf()
    plt.imshow(
        grid.grid.T,
        cmap='gray',
        origin='lower',
        extent=[
            -GRID_HALF*GRID_RES, GRID_HALF*GRID_RES,
            -GRID_HALF*GRID_RES, GRID_HALF*GRID_RES
        ]
    )
    plt.plot(odom.x, odom.y, 'bo', markersize=8)
    plt.arrow(
        odom.x, odom.y,
        0.1*math.cos(odom.th), 0.1*math.sin(odom.th),
        head_width=0.05, head_length=0.05, fc='b', ec='b'
    )
    if path:
        wx = [(cx - GRID_HALF) * GRID_RES for (cx, cy) in path]
        wy = [(cy - GRID_HALF) * GRID_RES for (cx, cy) in path]
        plt.plot(wx, wy, 'r.-')
    if goal:
        plt.plot(goal[0], goal[1], 'gx', markersize=10, markeredgewidth=2)
    plt.xlim(-GRID_HALF*GRID_RES, GRID_HALF*GRID_RES)
    plt.ylim(-GRID_HALF*GRID_RES, GRID_HALF*GRID_RES)
    plt.pause(0.001)

# =========================
# Main integration
# =========================
def main():
    # Parse CLI options for loading/saving maps and goal coordinates
    parser = argparse.ArgumentParser(description="Self driving robot with mapping/navigation")
    parser.add_argument("--load-map", "-l", help="Load occupancy grid from .npz file", default=None)
    parser.add_argument("--save-map", "-s", help="Save occupancy grid to .npz file on exit", default=None)
    parser.add_argument("--goal", "-g", nargs=2, type=float, metavar=("X", "Y"),
                        help="Goal coordinates in meters (x y)", default=[1.0, 1.0])
    parser.add_argument("--export-map-png", help="Export current map as a PNG image on exit", default=None)
    parser.add_argument("--export-ros-map", nargs=2, metavar=("PGM", "YAML"),
                        help="Export map in ROS format (pgm_path yaml_path)", default=None)
    parser.add_argument("--inflate-radius", type=float, help="Inflate obstacles by this radius (meters) before planning", default=0.0)
    parser.add_argument("--interactive-goal", action='store_true', help="Allow clicking on the visualization to set goal")
    parser.add_argument("--autosave-interval", type=float, help="Autosave map every N seconds (requires --save-map)", default=10.0)
    args = parser.parse_args()

    # register signal handlers so Ctrl+C (SIGINT) or SIGTERM trigger a clean shutdown
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    bus = init_can()
    odom = Odom()

    # Load map if requested, otherwise start with an empty grid
    if args.load_map:
        try:
            grid = Grid.load(args.load_map)
            print(f"Loaded map from {args.load_map}")
        except Exception as e:
            print(f"Failed to load map '{args.load_map}': {e}; creating a new empty grid")
            grid = Grid()
    else:
        grid = Grid()

    lidar = LidarThread()  # auto-detect port
    lidar.start()

    # Start autosave thread if requested
    autosave = None
    if args.save_map:
        autosave = AutosaveThread(grid, args.save_map, interval_s=args.autosave_interval)
        autosave.start()
        print(f"Autosave enabled: will save to {args.save_map} every {args.autosave_interval}s")

    goal_x, goal_y = float(args.goal[0]), float(args.goal[1])

    # If requested, inflate obstacles once before planning
    if args.inflate_radius and args.inflate_radius > 0.0:
        try:
            grid.inflate(args.inflate_radius)
            print(f"Inflated obstacles by {args.inflate_radius} m for planning")
        except Exception as e:
            print(f"Failed to inflate grid: {e}")

    # interactive goal setting via matplotlib click
    if args.interactive_goal:
        fig = plt.gcf()
        def _onclick(event):
            nonlocal goal_x, goal_y
            if event.inaxes is None:
                return
            gx = event.xdata
            gy = event.ydata
            if gx is None or gy is None:
                return
            goal_x, goal_y = float(gx), float(gy)
            print(f"Interactive goal set to {goal_x:.2f}, {goal_y:.2f}")
        try:
            fig.canvas.mpl_connect('button_press_event', _onclick)
        except Exception:
            # If running with non-interactive backend this will fail silently
            print("Interactive goal: could not attach click handler (non-interactive backend?)")

    # Persistent commanded velocities for ramping
    current_v = 0.0
    current_w = 0.0

    try:
        for nid in NODE_IDS:
            set_axis_state(bus, nid, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.2)

        target_v = 0.0
        target_w = 0.0
        last = time.time()
        print("Starting navigation loop...")

        while not STOP_EVENT.is_set():
            now = time.time()
            dt = max(1e-3, now - last)
            last = now

            # Get latest LIDAR scan and update occupancy grid
            scan = lidar.get_scan()
            for ang, rng in scan[::6]:  # subsample for speed
                grid.update_ray(odom.x, odom.y, odom.th + ang, rng)

            # Plan using A*
            start_cell = grid.world_to_cell(odom.x, odom.y)
            goal_cell  = grid.world_to_cell(goal_x, goal_y)
            path = astar(grid, start_cell, goal_cell)

            if path and len(path) > 1:
                # Convert path cells to world coordinates for pure pursuit
                path_world = [grid.cell_to_world(cx, cy) for (cx, cy) in path]
                target_v, target_w = pure_pursuit(path_world, odom, lookahead=0.35)
            else:
                v_react, w_react = reactive_plan(scan)
                target_v, target_w = v_react, w_react

            # Ramp commands towards targets
            v_diff = target_v - current_v
            w_diff = target_w - current_w
            if abs(v_diff) > VEL_RAMP:
                v_diff = VEL_RAMP * (1 if v_diff > 0 else -1)
            if abs(w_diff) > W_RAMP:
                w_diff = W_RAMP * (1 if w_diff > 0 else -1)
            current_v += v_diff
            current_w += w_diff
            current_v = max(-MAX_V, min(MAX_V, current_v))
            current_w = max(-MAX_W, min(MAX_W, current_w))

            # Convert to wheel linear velocities
            v_left  = current_v - 0.5 * BASELINE * current_w
            v_right = current_v + 0.5 * BASELINE * current_w

            # Convert to turns per second for motor controllers
            ts_left  = v_left  / (2.0 * math.pi * WHEEL_RADIUS)
            ts_right = v_right / (2.0 * math.pi * WHEEL_RADIUS)

            # Send commands over CAN
            set_velocity(bus, NODE_IDS[0], ts_left,  0.0)
            set_velocity(bus, NODE_IDS[1], ts_right, 0.0)

            # Update odometry
            odom.update(ts_left, ts_right, dt)

            # Visualize
            visualize(grid, odom, path=path, goal=(goal_x, goal_y))

    except KeyboardInterrupt:
        print("Shutting down (KeyboardInterrupt)...")
        STOP_EVENT.set()
    finally:
        # Stop autosave and lidar threads
        try:
            if 'autosave' in locals() and autosave:
                autosave.stop()
                autosave.join(timeout=2.0)
        except Exception:
            pass

        try:
            lidar.stop()
            lidar.join(timeout=2.0)
        except Exception:
            pass

        # Save map if requested on exit
        try:
            if 'args' in locals() and args.save_map:
                grid.save(args.save_map)
                print(f"Map saved to {args.save_map}")
        except Exception as e:
            print(f"Failed to save map to {args.save_map}: {e}")

        # Export ROS format map if requested
        try:
            if 'args' in locals() and args.export_ros_map:
                pgm_path, yaml_path = args.export_ros_map
                grid.export_ros_map(pgm_path, yaml_path)
                print(f"ROS map exported: {pgm_path} + {yaml_path}")
        except Exception as e:
            print(f"Failed to export ROS map: {e}")

        # Export PNG map if requested
        try:
            if 'args' in locals() and args.export_map_png:
                from PIL import Image
                ros_grid = np.full(grid.grid.shape, 127, dtype=np.uint8)
                ros_grid[grid.grid == 1] = 0
                ros_grid[grid.grid == -1] = 254
                img = Image.fromarray(ros_grid.T, mode='L')
                img.save(args.export_map_png)
                print(f"Map PNG exported to {args.export_map_png}")
        except ImportError:
            if 'args' in locals() and args.export_map_png:
                print("PNG export requires Pillow (PIL). Install with: pip install Pillow")
        except Exception as e:
            print(f"Failed to export PNG map: {e}")

        # Set axes to idle (optional safety)
        try:
            for nid in NODE_IDS:
                set_axis_state(bus, nid, AXIS_STATE_IDLE)
        except Exception:
            pass

        # Shutdown CAN cleanly
        shutdown_can(bus)

if __name__ == "__main__":
    main()
