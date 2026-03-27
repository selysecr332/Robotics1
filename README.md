# Robotics / Estimation Homework Collection

This repository contains multiple homework assignments focused on:
- geometric perception,
- sensor fusion,
- state estimation,
- and path planning.

Implemented homework folders:
- `hw_1` - Hough transform + RANSAC geometry tasks
- `hw_2` - Linear Kalman Filter for distance estimation
- `hw_3` - EKF attitude estimation (Euler vs Quaternion)
- `hw_5` - Path planning comparison (A*, RRT, RRT*)

---

## Repository Structure

```text
home/
├─ hw_1/
│  ├─ hw1_1_hough_line_detector.py
│  ├─ hw1_2_ransac_plane_detector.py
│  ├─ requirements.txt
│  └─ README.md
├─ hw_2/
│  ├─ HW2_Linear_Kalman_Filter.ipynb
│  ├─ accelerometer_2026-03-11_11.06.55.csv
│  ├─ gps_20260311_114127.csv
│  └─ README.md
├─ hw_3/
│  ├─ HW3_EKF_Attitude_Estimation.ipynb
│  ├─ imu_log.csv
│  └─ README.md
└─ hw_5/
   ├─ HW5_Path_Planning.ipynb
   ├─ hw5_movingai_sample.map
   ├─ README.md
   └─ README_RU.md
```

---

## Homework Overview

## HW 1 (`hw_1`)
Two standalone scripts:
- **Hough line detector** for 2D wall-like structures from noisy points
- **RANSAC plane detector** for 3D ground segmentation

Typical outputs:
- `hw1_1_wall_detection.png`
- `hw1_2_ground_detection.png`

## HW 2 (`hw_2`)
Notebook-based **Linear Kalman Filter** that fuses smartphone accelerometer and GPS to estimate traveled distance over time.

Main idea:
- Predict with acceleration dynamics
- Correct with GPS distance measurements

## HW 3 (`hw_3`)
Notebook-based **EKF attitude estimation** comparison:
- Euler-angle EKF
- Quaternion EKF

Produces roll/pitch/yaw comparison plots and quaternion consistency diagnostics.

## HW 5 (`hw_5`)
Path-planning notebook comparing:
- `A*`
- `RRT`
- `RRT*`

Includes smoothing, metrics, and algorithm animations (`astar_expansion.gif`, `rrt_tree.gif`).

---

## Suggested Environment

Use Python 3.10+ and install common packages used across assignments:

```bash
pip install numpy matplotlib scipy pandas opencv-python pillow jupyter
```

For HW1 point-cloud loading from real `.pcd/.ply` files:

```bash
pip install open3d
```

---

## How To Work With This Repo

1. Open a homework folder.
2. Read its local `README.md`.
3. For notebooks (`.ipynb`), run cells top to bottom.
4. For scripts (`.py`), run from terminal in the corresponding folder.

---

## Notes

- The repository currently includes `hw_1`, `hw_2`, `hw_3`, and `hw_5`.
- `hw_5` has both English and Russian documentation.
