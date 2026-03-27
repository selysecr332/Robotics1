import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List



def cartesian_to_hough(x: np.ndarray, y: np.ndarray, theta_res: float = np.pi / 180) -> Tuple[np.ndarray, np.ndarray]:
    """
    Transform Cartesian coordinates (x, y) to Hough space (rho, theta).
    Line equation: rho = x*cos(theta) + y*sin(theta)

    Returns:
        rho: perpendicular distance from origin to line
        theta: angle of normal to line (radians)
    """
    # theta in [0, pi) is enough (rho can be negative for other half)
    thetas = np.arange(0, np.pi, theta_res)
    # rho = x*cos(theta) + y*sin(theta) for each (x,y) and each theta
    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    # shape: (n_points, n_thetas)
    rho = np.outer(x, cos_t) + np.outer(y, sin_t)
    return rho, thetas


def hough_to_line(rho: float, theta: float) -> Tuple[float, float]:

    if np.abs(np.sin(theta)) < 1e-9:
        # nearly vertical line: x = rho / cos(theta)
        return np.inf, rho / np.cos(theta)  # use inf to denote vertical
    slope = -np.cos(theta) / np.sin(theta)
    intercept = rho / np.sin(theta)
    return slope, intercept


# 2. HOUGH ACCUMULATOR 

def build_hough_accumulator(
    x: np.ndarray,
    y: np.ndarray,
    theta_res: float = np.pi / 360,
    rho_res: float = 1.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build the Hough accumulator array.
    Each point (x,y) votes for all (rho, theta) that correspond to lines passing through (x,y).

    Returns:
        accumulator: 2D array (n_rho, n_theta)
        rho_bins: 1D array of rho values
        theta_bins: 1D array of theta values
    """
    thetas = np.arange(0, np.pi, theta_res)
    # rho range: from -max_dist to +max_dist
    max_rho = np.sqrt(np.max(x)**2 + np.max(y)**2) + 1
    rho_bins = np.arange(-max_rho, max_rho, rho_res)
    n_rho, n_theta = len(rho_bins), len(thetas)
    accumulator = np.zeros((n_rho, n_theta))

    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)
    for i in range(len(x)):
        rho_vals = x[i] * cos_t + y[i] * sin_t
        for j in range(n_theta):
            rho_idx = int(np.round((rho_vals[j] - rho_bins[0]) / rho_res))
            if 0 <= rho_idx < n_rho:
                accumulator[rho_idx, j] += 1

    return accumulator, rho_bins, thetas


#3  FIND PEAKS

def find_peaks(
    accumulator: np.ndarray,
    rho_bins: np.ndarray,
    theta_bins: np.ndarray,
    num_peaks: int = 4,
    min_distance_rho: int = 5,
    min_distance_theta: int = 5,
) -> List[Tuple[float, float]]:
    """
    Find peaks in the Hough accumulator (local maxima).
    Returns list of (rho, theta) for the top num_peaks lines.
    """
    try:
        from scipy.ndimage import maximum_filter
        neighborhood = np.ones((min_distance_rho * 2 + 1, min_distance_theta * 2 + 1))
        local_max = maximum_filter(accumulator, footprint=neighborhood) == accumulator
    except ImportError:
        local_max = np.ones_like(accumulator, dtype=bool)

    threshold = max(0.1 * np.max(accumulator), 3)
    peak_mask = local_max & (accumulator >= threshold)
    rho_idxs, theta_idxs = np.where(peak_mask)
    votes = accumulator[rho_idxs, theta_idxs]
    order = np.argsort(-votes)
    peaks = []
    rho_step = rho_bins[1] - rho_bins[0] if len(rho_bins) > 1 else 1.0
    theta_step = theta_bins[1] - theta_bins[0] if len(theta_bins) > 1 else 0.01
    for idx in order[: num_peaks * 3]:
        r, t = rho_idxs[idx], theta_idxs[idx]
        rho_val = rho_bins[r]
        theta_val = theta_bins[t]
        ok = True
        for (r0, t0) in peaks:
            if abs(rho_val - r0) < min_distance_rho * rho_step and \
               abs(theta_val - t0) < min_distance_theta * theta_step:
                ok = False
                break
        if ok:
            peaks.append((rho_val, theta_val))
        if len(peaks) >= num_peaks:
            break
    return peaks[:num_peaks]



def generate_room_points(width: float = 10.0, height: float = 8.0, points_per_edge: int = 50, noise_std: float = 0.08) -> Tuple[np.ndarray, np.ndarray]:
    """Generate synthetic 2D LIDAR-like points: rectangle (room contour) with Gaussian noise."""
    edges = [
        (np.linspace(0, width, points_per_edge), np.zeros(points_per_edge)),   # y=0
        (np.full(points_per_edge, width), np.linspace(0, height, points_per_edge)),
        (np.linspace(width, 0, points_per_edge), np.full(points_per_edge, height)),
        (np.zeros(points_per_edge), np.linspace(height, 0, points_per_edge)),
    ]
    x_list, y_list = [], []
    for xx, yy in edges:
        x_list.append(xx + np.random.normal(0, noise_std, len(xx)))
        y_list.append(yy + np.random.normal(0, noise_std, len(yy)))
    x = np.concatenate(x_list)
    y = np.concatenate(y_list)
    return x, y


def draw_line(ax, rho: float, theta: float, x_lim: Tuple[float, float], y_lim: Tuple[float, float], color: str = 'r', linewidth: float = 2):
    """Draw line rho = x*cos(theta)+y*sin(theta) in given axis limits."""
    if np.abs(np.sin(theta)) < 1e-9:
        x_line = np.array([rho / np.cos(theta), rho / np.cos(theta)])
        y_line = np.array([y_lim[0], y_lim[1]])
    else:
        x_line = np.array([x_lim[0], x_lim[1]])
        y_line = (rho - x_line * np.cos(theta)) / np.sin(theta)
    ax.plot(x_line, y_line, color=color, linewidth=linewidth, label='Detected line')


def main():
    # Generate synthetic room points
    np.random.seed(42)
    x, y = generate_room_points(width=10.0, height=8.0, points_per_edge=60, noise_std=0.06)

    # Build accumulator (use finer resolution for better accuracy)
    acc, rho_bins, theta_bins = build_hough_accumulator(x, y, theta_res=np.pi / 360, rho_res=0.05)
    peaks = find_peaks(acc, rho_bins, theta_bins, num_peaks=4, min_distance_rho=8, min_distance_theta=8)

    # Plot
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    ax.scatter(x, y, s=5, c='blue', alpha=0.7, label='LIDAR points')
    x_lim = (min(x) - 0.5, max(x) + 0.5)
    y_lim = (min(y) - 0.5, max(y) + 0.5)
    colors = ['red', 'green', 'orange', 'purple']
    for i, (rho, theta) in enumerate(peaks):
        draw_line(ax, rho, theta, x_lim, y_lim, color=colors[i % len(colors)])
    ax.set_xlim(x_lim)
    ax.set_ylim(y_lim)
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('1.1 Wall Detection: Hough Line Detector (2D LIDAR)')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    plt.tight_layout()
    plt.savefig('hw1_1_wall_detection.png', dpi=150)
    print("Detected lines (rho, theta):", peaks)
    plt.show()


if __name__ == '__main__':
    main()
