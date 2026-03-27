# HW 2: Estimating Travel Distance with a Linear Kalman Filter

This homework fuses smartphone **accelerometer** and **GPS** measurements to estimate traveled distance over time using a **1D linear Kalman filter**.

## Goal

Estimate distance more robustly than raw GPS by combining:
- High-rate accelerometer data (good short-term dynamics, noisy drift)
- Low-rate GPS position/distance data (absolute reference, noisy in urban/outdoor conditions)

## Folder Contents

- `HW2_Linear_Kalman_Filter.ipynb` - full implementation and plots
- `accelerometer_2026-03-11_11.06.55.csv` - phone accelerometer log
- `gps_20260311_114127.csv` - GPS log
- `README.md` - this file

## Method Summary

### 1) Data loading and preprocessing
- Load accelerometer CSV and convert values from `g` to `m/s^2`
- Load GPS CSV and convert latitude/longitude to distance from start with Haversine formula
- Align both signals to a shared time axis

### 2) Noise estimation
- `std_acc` (process noise): estimated from near-stationary accelerometer segment
- `std_meas` (measurement noise): estimated from GPS accuracy column or fallback heuristic

### 3) Linear Kalman Filter model
- State: `x = [position, velocity]^T`
- Control input: scalar acceleration along estimated motion direction
- Prediction:
  - `x_k|k-1 = F x_k-1|k-1 + B u_k`
  - `P_k|k-1 = F P_k-1|k-1 F^T + Q`
- Update (when GPS measurement is available):
  - `z_k = H x_k + v_k`
  - Standard Kalman gain update for `x` and `P`

### 4) Output visualization
- Plot compares:
  - **Predicted distance (KF)**
  - **Measured distance (GPS)**

## How to Run

1. Install requirements (typical):
   - `numpy`
   - `pandas`
   - `matplotlib`
   - `scipy`
2. Open `HW2_Linear_Kalman_Filter.ipynb` in Jupyter/Colab.
3. Run all cells top to bottom.

## Expected Result

You should obtain a smoother, physically consistent distance estimate that follows GPS trend while reducing raw GPS noise spikes.
