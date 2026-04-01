from dataclasses import dataclass
from typing import List, Optional, Tuple
import numpy as np

@dataclass
class CenterlineData:
    """Represents a centerline."""
    points: np.ndarray       # Nx2 array (x,y)
    curvature: np.ndarray    # N array (kappa)
    s: np.ndarray            # N array (arc length)
    # optional: spline representation (if needed)

@dataclass
class TrackData:
    """Represents a track with walls."""
    centerline: CenterlineData
    left_boundary: np.ndarray   # Nx2 array
    right_boundary: np.ndarray  # Nx2 array
    width_profile: np.ndarray    # N array (w(s))
    # optional: other attributes

@dataclass
class AirDuctsData:
    """Represents placed air ducts."""
    obbs: List[Tuple[np.ndarray, float, float]]  # each: (center, length, width)
    # or other representation (e.g., occupancy grid)
    occupancy_grid: Optional[np.ndarray] = None   # for .png output