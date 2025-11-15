# ROBOT_1_RELEASE_GOLD_11_15_2025
Self-Driving Robot (mapping + navigation)
========================================

This project demonstrates a self-driving robot stack with LIDAR-based mapping,
A* path planning, and pure-pursuit trajectory control. The robot builds an
occupancy grid in real-time, plans collision-free paths, and follows them smoothly.

Key features
============

- **Probabilistic occupancy mapping**: Log-odds based inverse sensor model with
  tunable hit/miss updates and thresholds. Maps persist to disk (.npz format).
- **Circular obstacle inflation**: Creates a safety margin around obstacles for
  safe planning (no external dependencies).
- **A* path planning**: Optimal collision-free paths on the occupancy grid.
- **Pure-pursuit controller**: Smooth path following using curvature-based steering.
- **Interactive goal setting**: Click on the visualization to set goals (if supported
  by your display backend).
- **Persistent mapping**: Save maps on exit with `--save-map` or auto-save periodically
  with `--autosave-interval`.
- **ROS map export**: Export grid as PGM + YAML for use with ROS map_server.
- **PNG export**: Visualize your map as an image file.

Dependencies
============

- **Core**: numpy, matplotlib, python-can, rplidar, pyserial
- **Optional**: Pillow (PIL) for PNG/PGM export

Install optional dependency:

```bash
pip install Pillow
```

Quick start
===========

Basic run with default goal (1.0, 1.0):

```bash
python3 self_driving_robot.py
```

Load existing map and set a custom goal:

```bash
python3 self_driving_robot.py --load-map mymap.npz --goal 2.0 -1.0
```

Save map on exit:

```bash
python3 self_driving_robot.py --save-map mymap.npz
```

Autosave map every 5 seconds:

```bash
python3 self_driving_robot.py --save-map mymap.npz --autosave-interval 5.0
```

Inflate obstacles by 0.2 m for safe planning:

```bash
python3 self_driving_robot.py --inflate-radius 0.2 --save-map mymap.npz
```

Export map to ROS format and PNG on exit:

```bash
python3 self_driving_robot.py \
  --save-map mymap.npz \
  --export-ros-map mymap.pgm mymap.yaml \
  --export-map-png mymap.png
```

Interactive goal setting (click on plot):

```bash
python3 self_driving_robot.py --interactive-goal
```

Full example with all features:

```bash
python3 self_driving_robot.py \
  --load-map previous_map.npz \
  --save-map updated_map.npz \
  --autosave-interval 10.0 \
  --inflate-radius 0.25 \
  --goal 3.0 2.0 \
  --interactive-goal \
  --export-ros-map map.pgm map.yaml \
  --export-map-png map.png
```

CLI reference
=============

```
--load-map FILE, -l FILE              Load occupancy grid from .npz file
--save-map FILE, -s FILE              Save occupancy grid to .npz file on exit
--goal X Y, -g X Y                    Goal coordinates in meters (default: 1.0 1.0)
--inflate-radius R                    Inflate obstacles by R meters (default: 0.0)
--interactive-goal                    Enable click-to-set goal on visualization
--export-map-png FILE                 Export map as PNG image on exit
--export-ros-map PGM YAML             Export map in ROS format (PGM + YAML)
--autosave-interval S                 Autosave map every S seconds (default: 10.0)
```

Running tests
=============

Unit tests verify save/load, inflation, and ROS export:

```bash
python3 -m unittest tests/test_grid.py -v
```

Or run a specific test:

```bash
python3 -m unittest tests/test_grid.py.TestGridInflation -v
```

Architecture notes
==================

- **Grid**: Stores occupancy as both a discrete grid (int8: -1=free, 0=unknown, 1=occ)
  and a continuous log-odds representation (float32) for incremental updates.
- **Mapping**: Each LIDAR scan updates the grid using an inverse sensor model.
  Rays are marked as "misses" (free) and endpoints as "hits" (occupied).
- **Planning**: A* searches the discrete grid for the shortest collision-free path.
- **Control**: Pure-pursuit steering follows the planned path, producing (v, w) commands.
- **Threads**: LIDAR thread continuously scans; autosave thread periodically saves the map.

Tuning parameters
=================

Edit these constants in `self_driving_robot.py` to tune behavior:

- `GRID_RES` (0.07 m): Cell resolution
- `GRID_SIZE` (200): Grid side length in cells
- `MAX_V`, `MAX_W`: Speed and angular velocity limits
- `VEL_RAMP`, `W_RAMP`: Acceleration limits
- `MIN_RANGE`, `MAX_RANGE`: LIDAR range filter
- In `Grid.__init__`: `hit_update`, `miss_update`, `occ_threshold`, `free_threshold`

Next improvements
==================

- Probabilistic beam model (ray casting with likelihood) for more robust mapping.
- Dynamic window approach (DWA) for local obstacle avoidance instead of reactive plan.
- FastSLAM or Kalman filter to jointly estimate odometry + map.
- Multi-goal planning and task scheduling.
- Looped closure detection for map refinement.

