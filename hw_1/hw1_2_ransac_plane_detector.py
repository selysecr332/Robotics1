
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from typing import Tuple, Optional


#PLANE FROM 3 POINTS

def plane_from_three_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Build plane equation ax + by + cz + d = 0 from 3 non-collinear points.
    Returns (normal [a,b,c], d) with normal unit length.
    """
    v1 = p2 - p1
    v2 = p3 - p1
    normal = np.cross(v1, v2)
    norm = np.linalg.norm(normal)
    if norm < 1e-10:
        raise ValueError("Collinear points: cannot define a unique plane.")
    normal = normal / norm
    d = -np.dot(normal, p1)
    return normal, d


def point_to_plane_distances(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    """Distance from each point to the plane |ax+by+cz+d| (assuming unit normal)."""
    # points: (N, 3), normal: (3,)
    return np.abs(points @ normal + d)


#  RANSAC 

def ransac_plane(
    points: np.ndarray,
    threshold: float = 0.02,
    max_iterations: int = 500,
    min_inliers_ratio: float = 0.1,
) -> Tuple[np.ndarray, float, np.ndarray]:
    """
    RANSAC for plane fitting (e.g. ground plane).

    Args:
        points: (N, 3) array of 3D points
        threshold: max distance from plane to count as inlier
        max_iterations: number of RANSAC iterations
        min_inliers_ratio: stop early if inlier ratio exceeds this

    Returns:
        best_normal: (3,) unit normal of best plane
        best_d: scalar d in ax+by+cz+d=0
        inlier_mask: (N,) boolean True for inliers (floor)
    """
    n_points = points.shape[0]
    best_inliers = np.zeros(n_points, dtype=bool)
    best_normal = np.array([0.0, 0.0, 1.0])
    best_d = 0.0

    for _ in range(max_iterations):
        # 1. Choose 3 random points
        idx = np.random.choice(n_points, size=3, replace=False)
        p1, p2, p3 = points[idx[0]], points[idx[1]], points[idx[2]]
        try:
            normal, d = plane_from_three_points(p1, p2, p3)
        except ValueError:
            continue
        # 2. Distances from all points to this plane
        dists = point_to_plane_distances(points, normal, d)
        # 3. Inliers: points within threshold
        inlier_mask = dists <= threshold
        n_inliers = np.sum(inlier_mask)
        if n_inliers > np.sum(best_inliers):
            best_inliers = inlier_mask
            best_normal = normal
            best_d = d
            if n_inliers >= n_points * min_inliers_ratio:
                # optional early stop
                break
    return best_normal, best_d, best_inliers

# LOAD POINT CLOUD 

def load_point_cloud(path: str) -> np.ndarray:
    """Load .pcd or .ply into (N, 3) numpy array. Uses Open3D if available."""
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(path)
        return np.asarray(pcd.points)
    except ImportError:
        # Minimal PCD reader (ASCII)
        if path.lower().endswith('.pcd'):
            return _load_pcd_ascii(path)
        raise FileNotFoundError("Install open3d: pip install open3d (or use synthetic data)")


def _load_pcd_ascii(path: str) -> np.ndarray:
    """Simple ASCII PCD loader."""
    with open(path, 'r') as f:
        lines = f.readlines()
    i = 0
    while i < len(lines) and not lines[i].startswith('DATA'):
        i += 1
    if i >= len(lines):
        raise ValueError("No DATA section in PCD")
    i += 1
    data = []
    while i < len(lines):
        parts = lines[i].split()
        if len(parts) >= 3:
            data.append([float(parts[0]), float(parts[1]), float(parts[2])])
        i += 1
    return np.array(data)


def generate_synthetic_pointcloud(
    floor_size: float = 5.0,
    n_floor: int = 2000,
    box_center: np.ndarray = None,
    box_half_size: float = 0.5,
    n_obstacle: int = 500,
    noise_std: float = 0.02,
) -> np.ndarray:
    """Generate 3D points: a floor (z=0) + a box obstacle, with noise."""
    if box_center is None:
        box_center = np.array([2.0, 2.0, 0.5])
    np.random.seed(42)
    # Floor: z ≈ 0
    floor_xy = np.random.uniform(-floor_size / 2, floor_size / 2, (n_floor, 2))
    floor_z = np.random.normal(0, noise_std, n_floor)
    floor_pts = np.column_stack([floor_xy[:, 0], floor_xy[:, 1], floor_z])
    # Box faces (simplified: points on box surface)
    obstacle_pts = []
    for _ in range(n_obstacle):
        face = np.random.randint(0, 6)
        u, v = np.random.uniform(-1, 1, 2)
        if face == 0:
            pt = box_center + np.array([u * box_half_size, v * box_half_size, box_half_size])
        elif face == 1:
            pt = box_center + np.array([u * box_half_size, v * box_half_size, -box_half_size])
        elif face == 2:
            pt = box_center + np.array([box_half_size, u * box_half_size, v * box_half_size])
        elif face == 3:
            pt = box_center + np.array([-box_half_size, u * box_half_size, v * box_half_size])
        elif face == 4:
            pt = box_center + np.array([u * box_half_size, box_half_size, v * box_half_size])
        else:
            pt = box_center + np.array([u * box_half_size, -box_half_size, v * box_half_size])
        obstacle_pts.append(pt + np.random.normal(0, noise_std, 3))
    obstacle_pts = np.array(obstacle_pts)
    return np.vstack([floor_pts, obstacle_pts])


# VISUALIZATION

def visualize_result(points: np.ndarray, inlier_mask: np.ndarray, title: str = "Ground Detection (RANSAC)"):
    """Plot point cloud: floor (one color), obstacles (other color)."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    floor_pts = points[inlier_mask]
    obst_pts = points[~inlier_mask]
    if len(floor_pts) > 0:
        ax.scatter(floor_pts[:, 0], floor_pts[:, 1], floor_pts[:, 2], c='green', s=1, alpha=0.6, label='Floor')
    if len(obst_pts) > 0:
        ax.scatter(obst_pts[:, 0], obst_pts[:, 1], obst_pts[:, 2], c='red', s=1, alpha=0.6, label='Obstacles')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    ax.legend()
    plt.tight_layout()
    plt.savefig('hw1_2_ground_detection.png', dpi=150)
    plt.show()


def main():
    import sys
    if len(sys.argv) > 1:
        path = sys.argv[1]
        print(f"Loading point cloud from {path}")
        points = load_point_cloud(path)
    else:
        print("No file provided. Using synthetic point cloud.")
        points = generate_synthetic_pointcloud(
            floor_size=6.0, n_floor=3000, n_obstacle=800, noise_std=0.02
        )

    if len(points) > 50000:
        idx = np.random.choice(len(points), 50000, replace=False)
        points = points[idx]

    print(f"Point cloud size: {points.shape[0]}")

    # RANSAC
    threshold = 0.03  
    normal, d, inlier_mask = ransac_plane(points, threshold=threshold, max_iterations=800)
    n_in = np.sum(inlier_mask)
    print(f"Plane: n·x + d = 0 with n = {normal}, d = {d}")
    print(f"Inliers (floor): {n_in} / {len(points)} ({100*n_in/len(points):.1f}%)")

    visualize_result(points, inlier_mask, "1.2 Ground Detection (RANSAC)")


if __name__ == '__main__':
    main()
