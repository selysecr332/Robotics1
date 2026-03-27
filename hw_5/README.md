# HW 5: Path Planning (A* vs RRT vs RRT*)

This homework implements and compares three path planners on occupancy-grid maps:

- `A*` (grid search, 8-connected)
- `RRT` (sampling-based planner)
- `RRT*` (RRT with rewiring for better path quality)

After planning, paths are smoothed and evaluated using runtime and path-quality metrics.

Main notebook:
- `HW5_Path_Planning.ipynb`

## Folder Contents

- `HW5_Path_Planning.ipynb` - full implementation, experiments, and plots
- `hw5_movingai_sample.map` - sample map in MovingAI format
- `README.md` - English description
- `README_RU.md` - Russian description

## What Is Implemented

- Occupancy-map loading from multiple sources
- Optional obstacle inflation for robot footprint safety
- `A*` planner with explored-cell snapshots for animation
- `RRT` planner with tree-growth snapshots
- `RRT*` planner with neighborhood search and rewiring
- Path smoothing:
  - gradient-based smoothing
  - collision-aware shortcut smoothing
- Metrics collection:
  - path length
  - runtime (seconds)
  - explored/visited nodes
- A* resolution study (downsampling effect)

## Map Sources

Switch map source in notebook variable `MAP_SOURCE`:

- `png` - occupancy map images (if provided)
- `movingai` - MovingAI `.map` files (sample included)
- `random_pr` - randomly generated rectangular obstacles

Occupancy convention:
- `grid[y, x] = 0` -> free cell
- `grid[y, x] = 1` -> obstacle cell

## Outputs

Notebook generates:
- Visual plots of raw and smoothed trajectories
- Comparative metrics table
- `astar_expansion.gif` (A* exploration animation)
- `rrt_tree.gif` (RRT growth animation)
- A* runtime vs map-resolution chart

## How to Run

1. Install dependencies:
   - `numpy`
   - `matplotlib`
   - `opencv-python`
   - `Pillow`
2. Open `HW5_Path_Planning.ipynb` in Jupyter or Colab.
3. Set `MAP_SOURCE` in the map configuration cell.
4. Run all cells in order.

## Main Observations

- A* is usually strong on structured grids and gives predictable results.
- RRT is fast to explore but may produce less optimal paths.
- RRT* improves path quality through rewiring but often costs more runtime.
- Smoothing noticeably improves geometric quality of all planners.
