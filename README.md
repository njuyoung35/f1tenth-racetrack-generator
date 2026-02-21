# f1tenth-racetrack-generator

## Table of Contents

- [1. Introduction and Aims](#1-introduction-and-aims)
- [2. System Requirements](#2-system-requirements)
- [3. Input Parameters](#3-input-parameters)
- [4. Algorithms Explained](#4-algorithms-explained)
  - [4.1 Centerline Generation](#41-centerline-generation)
  - [4.2 Wall Generation](#42-wall-generation)
  - [4.3 Global Path Optimization](#43-global-path-optimization)
- [5. Output Format](#5-output-format)
- [6. Example and Usage](#6-example-and-usage)
- [7. Project Structure](#7-project-structure)
- [8. Future Work / Roadmap](#8-future-work--roadmap)
- [9. Troubleshooting](#9-troubleshooting)
- [10. References](#10-references)

---

## 1. Introduction and Aims

`f1tenth-racetrack-generator` is a tool for automatically generating racetracks for the F1Tenth autonomous racing competition. Real F1Tenth tracks are constructed using bendable ducts (plastic conduits) as walls. This tool produces tracks with the following characteristics:

- **Realistic F1Tenth track properties**: Reflects the use of bendable ducts as walls.
- **Space‑partitioning**: Generates a track that occupies a given rectangular space (unlike Formula 1 tracks).
- **Single‑loop structure**: Produces a single closed loop with no forks.
- **Minimised dead‑ends**: The shape may resemble a maze or brain folding but contains no complete dead‑ends.
- **Shared physical walls**: Left and right walls are not necessarily separate obstacles; one duct can serve multiple wall sections.

The project is designed in three main stages:

1. **Centerline generation** – creates a smooth, closed-loop centerline.
2. **Wall generation** – adds physical walls (ducts) around the centerline, respecting duct properties.
3. **Global path optimisation** – (future) computes an optimised racing line.

---

## 2. System Requirements

- Python 3.13+
- [uv](https://github.com/astral-sh/uv) (>= 0.9.17) for dependency management
- Dependencies listed in `pyproject.toml`:
  - `numpy`
  - `scipy`
  - `matplotlib`
  - `pyyaml`
  - `shapely`
  - `concave-hull`
  - `jsonschema`

---

## 3. Input Parameters

## 3. Input Parameters

All configuration is done via a YAML file (default: `conf/config.yaml`). 
The configuration schema is **the single source of truth** for all parameters, their defaults, and documentation.

### Configuration Schema

The complete schema with detailed descriptions, default values, and constraints is maintained in:
[`conf/config.schema.json`](conf/config.schema.json)

This JSON Schema file:
- 📋 **Documents** every parameter with descriptions
- 🎯 **Defines** default values
- 🔒 **Specifies** constraints (min/max, allowed values)
- 🔗 **Serves** as the validation source for the Python code

### Example Configuration

A minimal working configuration looks like:

```yaml
config:
  space:
    width: 20.0
    height: 10.0
  
  centerline:
    n_points: 60
    max_lloyd_iterations: 10
    curvature_threshold: 0.3
  
  walls:
    duct_thickness: 0.30
    min_track_width: 3.00
    width_model:
      components:
        - type: constant
          value_dist:
            type: uniform
            low: 1.0
            high: 2.0
        - type: sinusoidal
          amplitude_dist:
            type: halfnormal
            sigma: 0.3
          frequency_dist:
            type: uniform
            low: 1
            high: 5
          phase_dist:
            type: uniform
            low: 0
            high: 6.283
  
  output:
    visualize: true
    output_dir: "output"
    debug_output_dir: "debug"
```

<!--All configuration is done via a YAML file (default: `conf/config.yaml`). The main parameters are:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `width` | Width of the rectangular space (m) | `20.0` |
| `height` | Height of the rectangular space (m) | `10.0` |
| `n_points` | Number of initial random points | `60` |
| `max_lloyd_iterations` | Iterations for Lloyd's algorithm | `10` |
| `max_verifying_iterations` | Retries for valid centerline | `50` |
| `virtual_grid_width` | Cell size for grid‑based region selection (m) | `5.0` |
| `virtual_grid_coverage` | Fraction of grid cells to use | `0.8` |
| `curvature_threshold` | Maximum allowed curvature (1/m) | `0.3` |
| `visualize` | Enable debug visualisations | `false` |
| `debug_output_dir` | Directory for debug images | `"debug"` |
| **Wall‑generation parameters** (see section 4.2) | | |
| `duct_thickness` | Physical thickness of each duct (m) | `0.30` |
| `min_duct_length` | Minimum duct segment length (m) | `0.50` |
| `max_duct_length` | Maximum duct segment length (m) | `2.00` |
| `min_track_width` | Minimum allowed track width (m) | `3.00` |
| `wall_points_per_meter` | Density of generated wall points | `50` |
| `connection_tolerance` | Tolerance for duct connections (m) | `0.15` |
| `resolution` | Occupancy grid cell size (m) | `0.05` |-->

---

## 4. Algorithms Explained

### 4.1 Centerline Generation

**Status:** ✅ Almost complete (small refinements possible).

The centerline is generated using a Voronoi‑based approach that creates a smooth, closed loop with controlled curvature. The steps are:

1. **Generate initial random points** inside the rectangle (with a small margin).  
   Number of points = `n_points`.

2. **Run Lloyd’s algorithm** (for `max_lloyd_iterations`) to relax the points, making their spacing more uniform.  
   This produces a Centroidal Voronoi Tessellation (CVT). The `lloyd.py` module implements this.

3. **Select Voronoi regions using a virtual grid.**  
   - Divide the rectangle into cells of size `virtual_grid_width`.  
   - For each cell (with probability `virtual_grid_coverage`), consider the `k` nearest seed points (k randomly chosen from 3–5).  
   - Pick one seed point from these `k` with probability inversely proportional to distance (closer points are more likely).  
   - Collect selected region indices (remove duplicates).

4. **Build a concave hull** from the selected seed points.  
   - Use the `concave_hull` library with a fixed `concavity` value (currently 0.33).  
   - The hull becomes the initial polygon that approximates the track shape.

5. **Interpolate the polygon using Voronoi vertices** while respecting the curvature limit.  
   - For each polygon edge, collect nearby Voronoi vertices (within a distance proportional to edge length).  
   - Insert those vertices to make the path follow the Voronoi structure more closely.  
   - Then refine the point set by checking curvature: if maximum curvature exceeds `curvature_threshold`, remove the point causing the highest curvature and re‑interpolate (iteratively up to 10 times).

6. **Final interpolation** using a cubic spline (`scipy.interpolate.splprep` with `per=True` for closed curves).  
   The number of output points is scaled to achieve approximately 0.5 m spacing along the track (or finer if needed).

The result is a smooth, closed centerline that respects the curvature constraint and fills the space in an organic way.

### 4.2 Wall Generation

**Status:** 🚧 Under development / needs update.

This stage builds physical walls (ducts) around the centerline. The approach is **duct‑based**, inspired by how real F1Tenth tracks are built: a series of bendable ducts, each with a fixed thickness, are placed along the track. One duct can serve both the left and right walls in a given segment.

The algorithm proceeds as follows:

1. **Parse the centerline** into a list of `CenterlinePoint` objects, each containing:
   - `x, y` – coordinates
   - `t` – normalised parameter [0,1]
   - `cumulative_distance` – distance along the track
   - `curvature` – curvature at that point
   - `left_width`, `right_width` – maximum allowed raycast distances to the left/right (computed from the space boundaries and/or obstacles)

2. **Plan duct layout** by dividing the centerline into segments.  
   - The total track length is split into roughly equal segments whose lengths lie between `min_duct_length` and `max_duct_length`.  
   - Each segment will later become a `DuctSegment` object that stores its own portion of the centerline.

3. **Generate wall point clouds** using a **lateral offset function**.  
   For each centerline point, compute the left and right normals (perpendicular to the tangent). Then obtain an offset distance from a user‑supplied function:
   ```python
   def lateral_offset_fn(x, y, t, cumulative_distance, curvature,
                         left_width, right_width, base_width, is_left, **kwargs):
       # returns a float (offset distance)
       ...
   ```
   This function can implement arbitrary logic (e.g., constant width, sinusoidal variation, curvature‑dependent width).  
   The offset is clamped to `left_width` or `right_width` (to stay within the space).  
   For each point, a small cluster of points is generated around the nominal wall location to mimic sensor noise or duct flexibility (density controlled by `wall_points_per_meter`).

4. **Assign wall points to the nearest duct segment**.  
   - Build a KD‑tree of all wall points.  
   - For each duct segment, query points within a radius (segment length/2 + margin).  
   - Compute the perpendicular distance from each candidate point to the duct’s centerline; keep points within `1.5 * duct_thickness`.  
   - Store the assigned points in the duct segment.

5. **Create an occupancy grid** at resolution `resolution`.  
   - Determine the bounding box of all wall points (with margin).  
   - Rasterise the wall points into the grid, filling a circle of radius `duct_thickness/2` around each point.  
   - Optionally mark duct‑assigned points with a separate value for debugging.

6. **Validate the result**:
   - Enough wall points generated? (>100)
   - Most points assigned to a duct? (>80%)
   - Track width at every centerline point ≥ `min_track_width`? (using left/right raycast lengths).

The output includes:
- The full list of generated wall points.
- The occupancy grid.
- The list of duct segments with their assigned points.
- Metadata (statistics, parameters used).

### 4.3 Global Path Optimization

**Status:** 🚧 Planned (future work).

This module will take the generated track (centerline + walls) and compute an optimal racing line. The optimised path should be:
- Collision‑free (inside the track boundaries)
- Smooth (respect curvature limits of the vehicle)
- Possibly time‑optimal (minimum curvature / maximum speed)

The output will be a new set of points (N x 7) with the same format as the centerline (x, y, t, cumulative distance, curvature, left_width, right_width) but representing the racing line.

**Ideas for implementation:**
- Use an optimisation‑based approach (e.g., spline fitting with constraints).
- Employ the concept of “raceline” derived from the track boundaries (e.g., minimum curvature, clipping corners).
- Or implement a pure pursuit / model predictive control based path planning.

---

## 5. Output Format

After running the complete pipeline, the following files are saved in the `output/` directory:

| File | Description |
|------|-------------|
| `centerline.npy` | (N, 7) array: `[x, y, t, cumulative_distance, curvature, left_width, right_width]` |
| `wall_points.npy` | (M, 2) array of raw wall point coordinates |
| `wall.npy` | Occupancy grid (2D numpy array, values 0 = free, 1 = wall, optionally 2 = duct‑assigned) |
| `metadata.json` | Generation parameters and statistics |

The centerline format uses **Frenet‑style** coordinates: `t` is the normalised parameter (0 to 1), `cumulative_distance` is the actual distance along the track from a chosen start. The widths (`left_width`, `right_width`) are the maximum available space to the left/right at that point, derived from ray‑casting to the space boundaries (and later possibly to walls).

---

## 6. Example and Usage

1. **Clone the repository** and navigate to the project root.
2. **Install dependencies** using `uv`:
   ```bash
   uv sync
   ```
3. **Edit the configuration** in `conf/config.yaml` to your liking.
4. **Run the generator**:
   ```bash
   uv run python main.py
   ```
5. **Check the output** in the `output/` folder and debug images in the `debug/` folder (if `visualize: true`).

A typical `main.py` does:
```python
import yaml
from centerline import CenterlineGenerator
from walls import WallGenerator

config = yaml.safe_load(open("conf/config.yaml"))["config"]
config["visualize"] = True
config["debug_output_dir"] = "./debug"

cg = CenterlineGenerator(config)
centerline = cg.generate()               # returns Nx2 array (x,y)
# (metadata like cumulative distance is added later in main)

wg = WallGenerator(config)
result = wg.generate(centerline_with_metadata)   # returns dict with points, grid, etc.

# Save outputs...
```

---

## 7. Project Structure

```
.
├── conf
│   └── config.yaml       # Main configuration file
├── debug/                 # Debug visualisations (created at runtime)
├── output/                # Generated output files (created at runtime)
├── src
│   ├── centerline.py     # CenterlineGenerator class
│   └── walls.py          # WallGenerator class (duct‑based)
├── main.py                # Main entry point
├── pyproject.toml         # Project dependencies and metadata
└── README.md              # This file

├── track_width_profile_functions.py  # Example lateral offset functions
```

### Key Modules

- **centerline.py** – Implements the Voronoi‑based centerline generation.
- **walls.py** – Duct‑based wall generation (currently under development).
- **track_width_profile_functions.py** – Contains example lateral offset functions (constant, sinusoidal, chicane) to be used with wall generation.
- **main.py** – Orchestrates the pipeline and saves results.

---

## 8. Future Work / Roadmap

- **Complete and stabilise wall generation** (`walls.py`):
  - Integrate ray‑casting to obtain accurate left/right maximum widths.
  - Implement a robust duct connection mechanism.
  - Add more sophisticated lateral offset functions (e.g., learning from real track data).
- **Implement global path optimisation** (`global_path.py` or similar).
- **Add unit tests and CI**.
- **Provide a graphical user interface** (optional) for interactive track design.
- **Benchmark generated tracks** against real F1Tenth tracks.

---

## 9. Troubleshooting

- **`uv sync` fails** – ensure you have Python 3.13+ installed and `uv` is up‑to‑date.
<!--- **Visualisations not saved** – check that `visualize: true` in config and the debug directory is writable.-->
<!--- **`centerline` generation repeatedly fails** – try increasing `max_verifying_iterations` or adjusting `virtual_grid_coverage` / `curvature_threshold`.-->
<!--- **Wall points are too sparse** – increase `wall_points_per_meter` or reduce `resolution`.-->
<!--- **Track width too narrow** – adjust `min_track_width` or check that `left_width`/`right_width` from ray‑casting are sufficient.-->

---

## 10. References

- [Random Track Generator (mvanlobensels)](https://github.com/mvanlobensels/random-track-generator)
- [DBSCAN (Wikipedia)](https://en.wikipedia.org/wiki/DBSCAN)
- [Lloyd library by duhaime](https://github.com/duhaime/lloyd/tree/master)
- [concave‑hull library by cubao](https://github.com/cubao/concave_hull)
- Assisted by LLM: DeepSeek

---

**Note:** This README is a living document. As the project evolves, please update it to reflect the current state of development.
