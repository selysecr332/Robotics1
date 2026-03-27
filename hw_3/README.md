# HW 3: EKF Attitude Estimation (Euler vs Quaternion)

This homework compares two Extended Kalman Filter (EKF) approaches for IMU-based attitude estimation:

- **Variant A:** Euler-angle EKF
- **Variant B:** Quaternion EKF

The objective is to estimate and compare **roll, pitch, yaw** trajectories from accelerometer and gyroscope data.

## Folder Contents

- `HW3_EKF_Attitude_Estimation.ipynb` - main notebook with both EKF variants
- `imu_log.csv` - IMU dataset (`acc_*`, `gyro_*`)

## Problem Setup

Input sensor data:
- Accelerometer in `m/s^2`
- Gyroscope in `rad/s`

Both filters use:
- Gyroscope for prediction (state propagation)
- Accelerometer as gravity reference in update step

## Variant A: Euler-based EKF

- State: `x = [roll, pitch, yaw]^T`
- Prediction:
  - Uses Euler-rate matrix and gyro integration
- Update:
  - Compares measured acceleration to expected gravity in body frame
- Includes numerical stabilization in covariance update

## Variant B: Quaternion-based EKF

- State: `q = [qw, qx, qy, qz]^T`
- Prediction:
  - Quaternion kinematics with angular velocity
  - Explicit quaternion normalization each step
- Update:
  - Same gravity-consistency idea, but in quaternion form
- Tracks quaternion norm before normalization for consistency diagnostics

## Outputs in Notebook

- Roll / Pitch / Yaw comparison plots:
  - Euler EKF vs Quaternion EKF
- Quaternion norm plot (before normalization)
- Printed execution status for both variants

## How to Run

1. Install dependencies (typical):
   - `numpy`
   - `pandas`
   - `matplotlib`
2. Open `HW3_EKF_Attitude_Estimation.ipynb` in Jupyter/Colab.
3. Make sure `imu_log.csv` is accessible (same folder or uploaded in Colab).
4. Run all cells in order.

## Key Takeaway

Euler EKF is intuitive but can be more sensitive near singular configurations. Quaternion EKF usually provides better rotational consistency for 3D attitude tracking.
