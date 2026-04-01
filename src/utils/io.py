"""
Input/Output utilities for saving and loading track data, metrics, and generated assets.
"""

import os
import yaml
import numpy as np
from pathlib import Path
from PIL import Image
from typing import Dict, Any, Optional, Union
import json

# Import data structures (relative imports)
from ..core.dataclass import CenterlineData, TrackData
from ..air_duct.base import AirDuctsData, OrientedBoundingBox


def save_track_to_yaml(track: TrackData, air_ducts: AirDuctsData,
                       metrics: Dict[str, float], output_dir: Union[str, Path],
                       filename: str = "track_data.yaml") -> None:
    """
    Save track data, air duct info, and metrics to a YAML file.

    Args:
        track: TrackData object
        air_ducts: AirDuctsData object
        metrics: Dictionary of computed metrics
        output_dir: Directory to save the file
        filename: Name of the YAML file
    """
    output_path = Path(output_dir) / filename
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Convert numpy arrays to lists for YAML serialization
    data = {
        'track': track.to_dict(),
        'air_ducts': air_ducts.to_dict(),
        'metrics': metrics,
        'metadata': {
            'version': '1.0',
            'timestamp': str(Path(output_path).stat().st_ctime) if output_path.exists() else None
        }
    }

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)


def load_track_from_yaml(filepath: Union[str, Path]) -> Dict[str, Any]:
    """
    Load track data from a YAML file.

    Args:
        filepath: Path to YAML file

    Returns:
        Dictionary containing track, air_ducts, metrics
    """
    with open(filepath, 'r') as f:
        data = yaml.safe_load(f)

    # Convert lists back to numpy arrays where needed
    track_dict = data['track']
    track_dict['centerline'] = CenterlineData.from_dict(track_dict['centerline'])
    track_dict['left_boundary'] = np.array(track_dict['left_boundary'])
    track_dict['right_boundary'] = np.array(track_dict['right_boundary'])
    track_dict['width_profile'] = np.array(track_dict['width_profile'])
    track = TrackData(**track_dict)

    # Reconstruct air ducts
    air_ducts_dict = data['air_ducts']
    obbs = []
    for obb_dict in air_ducts_dict.get('obbs', []):
        obb = OrientedBoundingBox(
            center=np.array(obb_dict['center']),
            length=obb_dict['length'],
            width=obb_dict['width'],
            angle=obb_dict['angle'],
            corners=np.array(obb_dict['corners']) if obb_dict.get('corners') else None
        )
        obbs.append(obb)
    air_ducts = AirDuctsData(
        obbs=obbs,
        occupancy_grid=np.array(air_ducts_dict['occupancy_grid']) if air_ducts_dict.get('occupancy_grid') is not None else None,
        grid_resolution=air_ducts_dict.get('grid_resolution', 0.05),
        grid_size=tuple(air_ducts_dict['grid_size']) if air_ducts_dict.get('grid_size') else None,
        representation=air_ducts_dict['representation'],
        metadata=air_ducts_dict.get('metadata', {})
    )

    metrics = data['metrics']

    return {'track': track, 'air_ducts': air_ducts, 'metrics': metrics}


def save_occupancy_grid(track: TrackData, air_ducts: AirDuctsData,
                        output_dir: Union[str, Path],
                        resolution: float = 0.05,
                        filename: str = "occupancy_grid.png") -> None:
    """
    Generate occupancy grid from air ducts and track walls, and save as PNG.

    The grid is a binary image where:
        1 = occupied (wall or air duct)
        0 = free space

    Args:
        track: TrackData object
        air_ducts: AirDuctsData object
        output_dir: Output directory
        resolution: Grid resolution in meters per pixel
        filename: Name of the output PNG file
    """
    # Determine world bounds from track boundaries
    all_points = np.vstack([track.left_boundary, track.right_boundary])
    x_min, y_min = all_points.min(axis=0) - 1.0  # add margin
    x_max, y_max = all_points.max(axis=0) + 1.0

    # Grid dimensions
    width_m = x_max - x_min
    height_m = y_max - y_min
    nx = int(width_m / resolution) + 1
    ny = int(height_m / resolution) + 1

    # Initialize grid: 0 = free
    grid = np.zeros((ny, nx), dtype=np.uint8)

    # Function to map world coordinates to grid indices
    def world_to_grid(x, y):
        ix = int((x - x_min) / resolution)
        iy = int((y - y_min) / resolution)
        return ix, iy

    # Mark walls (track boundaries)
    # For simplicity, we treat the area between left and right boundaries as free,
    # and the area outside as occupied. Here we mark the walls themselves.
    # More accurate: fill the interior as free and exterior as occupied,
    # but for occupancy grid we typically mark obstacles.
    # We'll mark points along the boundaries.
    for boundary in [track.left_boundary, track.right_boundary]:
        for point in boundary:
            ix, iy = world_to_grid(point[0], point[1])
            if 0 <= ix < nx and 0 <= iy < ny:
                grid[iy, ix] = 1

    # Mark air ducts (if OBBs exist)
    for obb in air_ducts.obbs:
        # Get bounding box of OBB
        min_x = min(obb.corners[:, 0])
        max_x = max(obb.corners[:, 0])
        min_y = min(obb.corners[:, 1])
        max_y = max(obb.corners[:, 1])

        ix_min = max(0, int((min_x - x_min) / resolution))
        ix_max = min(nx, int((max_x - x_min) / resolution) + 1)
        iy_min = max(0, int((min_y - y_min) / resolution))
        iy_max = min(ny, int((max_y - y_min) / resolution) + 1)

        for iy in range(iy_min, iy_max):
            y = y_min + iy * resolution
            for ix in range(ix_min, ix_max):
                x = x_min + ix * resolution
                if obb.contains_point(np.array([x, y])):
                    grid[iy, ix] = 1

    # Save as PNG (invert colors for typical map: black=free, white=occupied)
    img = Image.fromarray((1 - grid) * 255)  # invert: 0->255, 1->0
    output_path = Path(output_dir) / filename
    img.save(output_path)

    # Also save a YAML descriptor for the map (like ROS map)
    map_yaml = {
        'image': filename,
        'resolution': resolution,
        'origin': [x_min, y_min, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'negate': 0
    }
    map_yaml_path = Path(output_dir) / "map.yaml"
    with open(map_yaml_path, 'w') as f:
        yaml.dump(map_yaml, f, default_flow_style=False)


def save_metrics_csv(metrics: Dict[str, float], output_dir: Union[str, Path],
                     filename: str = "metrics.csv") -> None:
    """
    Save metrics to a CSV file (one row).

    Args:
        metrics: Dictionary of metric names to values
        output_dir: Output directory
        filename: Name of the CSV file
    """
    output_path = Path(output_dir) / filename
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Write header if file doesn't exist
    if not output_path.exists():
        with open(output_path, 'w') as f:
            f.write(','.join(metrics.keys()) + '\n')
            f.write(','.join(str(v) for v in metrics.values()) + '\n')
    else:
        with open(output_path, 'a') as f:
            f.write(','.join(str(v) for v in metrics.values()) + '\n')


def save_to_usd(track: TrackData, air_ducts: AirDuctsData,
                output_dir: Union[str, Path],
                filename: str = "track.usda") -> None:
    """
    Save track and air ducts as a USD (Universal Scene Description) file.

    This is a simplified USD export that can be extended later.
    It creates a basic .usda file with a mesh for the track surface and
    boxes for air ducts. Requires the 'usd-core' package.

    Args:
        track: TrackData object
        air_ducts: AirDuctsData object
        output_dir: Output directory
        filename: Name of the USD file
    """
    try:
        from pxr import Usd, UsdGeom, Gf, Sdf
    except ImportError:
        print("USD library not available. Install 'usd-core' for USD export.")
        return

    output_path = Path(output_dir) / filename
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Create stage
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # Add a mesh for the track surface (a simple plane or extruded shape)
    # For simplicity, create a plane mesh from the track boundaries.
    # This is a placeholder; full track mesh requires triangulation.

    # Example: Create a plane at z=0 with texture
    plane = UsdGeom.Mesh.Define(stage, Sdf.Path("/TrackPlane"))
    plane.CreatePointsAttr([(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 2, 3, 0])
    plane.CreateFaceVertexCountsAttr([3, 3])

    # Add air ducts as cubes
    for i, obb in enumerate(air_ducts.obbs):
        cube = UsdGeom.Cube.Define(stage, Sdf.Path(f"/AirDuct_{i}"))
        cube.GetSizeAttr().Set(obb.length)  # Note: cube size is uniform; for OBB we need a transform
        # Set transform: scale to length, width, height; then rotate and translate
        xform = UsdGeom.XformCommonAPI(cube)
        # For OBB, we need a non-uniform scale
        # This is simplified: we just set translation and orientation.
        xform.SetTranslate((obb.center[0], obb.center[1], 0.1))
        xform.SetRotate((0, 0, np.degrees(obb.angle)))  # rotate around Z

    # Save stage
    stage.GetRootLayer().Save()


def save_all_outputs(track: TrackData, air_ducts: AirDuctsData,
                     metrics: Dict[str, float], output_dir: Union[str, Path],
                     config: Optional[Dict[str, Any]] = None) -> None:
    """
    Convenience function to save all outputs (YAML, occupancy grid, USD, metrics).

    Args:
        track: TrackData
        air_ducts: AirDuctsData
        metrics: metrics dict
        output_dir: output directory
        config: optional configuration used for generation (saved as config.yaml)
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Save YAML
    save_track_to_yaml(track, air_ducts, metrics, output_dir)

    # Save occupancy grid
    save_occupancy_grid(track, air_ducts, output_dir)

    # Save USD (optional)
    save_to_usd(track, air_ducts, output_dir)

    # Save metrics CSV
    save_metrics_csv(metrics, output_dir)

    # Save configuration if provided
    if config is not None:
        config_path = output_dir / "config.yaml"
        with open(config_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)