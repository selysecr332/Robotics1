# HW 1: Feature Extraction and Geometric Fitting

This homework contains two independent computer vision / robotics geometry tasks:

1. **2D wall detection** from synthetic LiDAR-like points using the **Hough Transform**
2. **3D ground plane detection** from point clouds using **RANSAC**

## Folder Contents

- `hw1_1_hough_line_detector.py` - Hough-based line extraction in 2D
- `hw1_2_ransac_plane_detector.py` - RANSAC plane fitting in 3D
- `requirements.txt` - Python dependencies for HW1

## Task 1: Hough Line Detector

The script:
- Generates a noisy rectangular room contour (`generate_room_points`)
- Builds a Hough accumulator over `(rho, theta)` (`build_hough_accumulator`)
- Finds strong peaks as wall candidates (`find_peaks`)
- Draws detected lines on top of point data

Output:
- Figure saved as `hw1_1_wall_detection.png`

Run:

```bash
python hw1_1_hough_line_detector.py
```

## Task 2: RANSAC Plane Detector

The script:
- Loads a point cloud from `.pcd` / `.ply` (Open3D if available)
- Falls back to synthetic data if no file is provided
- Uses RANSAC to estimate plane `ax + by + cz + d = 0`
- Classifies inliers as floor and outliers as obstacles
- Visualizes result in 3D

Output:
- Figure saved as `hw1_2_ground_detection.png`

Run with synthetic cloud:

```bash
python hw1_2_ransac_plane_detector.py
```

Run with your own point cloud:

```bash
python hw1_2_ransac_plane_detector.py path/to/pointcloud.pcd
```

## Dependencies

Install:

```bash
pip install -r requirements.txt
```

`open3d` is only required when reading real `.pcd` / `.ply` files.

## Notes

- Task 1 is fully synthetic and reproducible via random seed.
- Task 2 supports both synthetic testing and real sensor point-cloud input.
