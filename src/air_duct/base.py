"""
Base classes and data structures for air duct placement.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List, Tuple, Union
import numpy as np
from enum import Enum


class AirDuctRepresentation(Enum):
    """Enum for air duct representation types."""
    OBB = "obb"  # Oriented Bounding Boxes
    POINTS = "points"
    OCCUPANCY_GRID = "occupancy_grid"


@dataclass
class OrientedBoundingBox:
    """
    Data class for oriented bounding box.
    
    Attributes:
        center: 2D center point
        length: Length along the primary axis
        width: Width perpendicular to primary axis
        angle: Orientation angle (radians)
        corners: 4x2 array of corner points (optional)
    """
    center: np.ndarray  # 2
    length: float
    width: float
    angle: float
    corners: Optional[np.ndarray] = None  # 4x2
    
    def __post_init__(self):
        """Compute corners if not provided."""
        if self.corners is None:
            self.corners = self.compute_corners()
    
    def compute_corners(self) -> np.ndarray:
        """Compute the four corners of the OBB."""
        # Local coordinates (axis-aligned)
        half_length = self.length / 2.0
        half_width = self.width / 2.0
        
        corners_local = np.array([
            [-half_length, -half_width],
            [ half_length, -half_width],
            [ half_length,  half_width],
            [-half_length,  half_width]
        ])
        
        # Rotation matrix
        cos_theta = np.cos(self.angle)
        sin_theta = np.sin(self.angle)
        rotation = np.array([[cos_theta, -sin_theta],
                            [sin_theta,  cos_theta]])
        
        # Transform to global coordinates
        corners = corners_local @ rotation.T + self.center
        return corners
    
    def contains_point(self, point: np.ndarray, margin: float = 0.0) -> bool:
        """Check if point is inside the OBB."""
        # Transform point to local coordinates
        cos_theta = np.cos(self.angle)
        sin_theta = np.sin(self.angle)
        
        # Translation to origin
        p = point - self.center
        
        # Rotation to align with axes
        x_local = p[0] * cos_theta + p[1] * sin_theta
        y_local = -p[0] * sin_theta + p[1] * cos_theta
        
        # Check bounds
        half_length = self.length / 2.0 + margin
        half_width = self.width / 2.0 + margin
        
        return (abs(x_local) <= half_length and abs(y_local) <= half_width)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            'center': self.center.tolist(),
            'length': self.length,
            'width': self.width,
            'angle': self.angle,
            'corners': self.corners.tolist() if self.corners is not None else None
        }


@dataclass
class AirDuctsData:
    """
    Data class for air duct placement.
    
    Attributes:
        obbs: List of oriented bounding boxes
        occupancy_grid: Optional occupancy grid for .png output
        grid_resolution: Resolution of occupancy grid (pixels per meter)
        grid_size: Size of grid (height, width)
        representation: Type of representation
        metadata: Additional metadata
    """
    obbs: List[OrientedBoundingBox] = field(default_factory=list)
    occupancy_grid: Optional[np.ndarray] = None  # 2D binary array
    grid_resolution: float = 0.05  # 5 cm per pixel (typical for F1Tenth)
    grid_size: Optional[Tuple[int, int]] = None
    representation: AirDuctRepresentation = AirDuctRepresentation.OBB
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def add_obb(self, center: np.ndarray, length: float, width: float, angle: float):
        """Add a new OBB."""
        self.obbs.append(OrientedBoundingBox(center, length, width, angle))
    
    def to_occupancy_grid(self, width: float, height: float) -> np.ndarray:
        """
        Convert OBBs to occupancy grid.
        
        Args:
            width: Track width (x-direction) in meters
            height: Track height (y-direction) in meters
            
        Returns:
            2D numpy array where 1 = occupied (duct), 0 = free
        """
        # Grid dimensions
        nx = int(width / self.grid_resolution) + 1
        ny = int(height / self.grid_resolution) + 1
        grid = np.zeros((ny, nx), dtype=np.uint8)
        
        # Fill grid with OBBs
        for obb in self.obbs:
            # Get bounding box for efficient iteration
            min_x = min(obb.corners[:, 0])
            max_x = max(obb.corners[:, 0])
            min_y = min(obb.corners[:, 1])
            max_y = max(obb.corners[:, 1])
            
            # Convert to grid indices
            ix_min = max(0, int(min_x / self.grid_resolution))
            ix_max = min(nx, int(max_x / self.grid_resolution) + 1)
            iy_min = max(0, int(min_y / self.grid_resolution))
            iy_max = min(ny, int(max_y / self.grid_resolution) + 1)
            
            # Check each grid cell
            for iy in range(iy_min, iy_max):
                y = iy * self.grid_resolution
                for ix in range(ix_min, ix_max):
                    x = ix * self.grid_resolution
                    if obb.contains_point(np.array([x, y])):
                        grid[iy, ix] = 1
        
        self.occupancy_grid = grid
        self.grid_size = (ny, nx)
        return grid
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'obbs': [obb.to_dict() for obb in self.obbs],
            'grid_resolution': self.grid_resolution,
            'grid_size': self.grid_size,
            'representation': self.representation.value,
            'metadata': self.metadata
        }


class AirDuctPlacer(ABC):
    """
    Abstract base class for air duct placement algorithms.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        """
        Initialize air duct placer.
        
        Args:
            config: Configuration dictionary
            seed: Random seed
        """
        self.config = config
        self.seed = seed
        if seed is not None:
            np.random.seed(seed)
        
        # Common parameters
        self.duct_length = config.get('duct_length', 0.4)  # meters
        self.duct_width = config.get('duct_width', 0.1)   # meters
        self.gap_tolerance = config.get('gap_tolerance', 0.02)  # meters
    
    @abstractmethod
    def place(self, track: Any) -> AirDuctsData:
        """
        Place air ducts along the track.
        
        Args:
            track: TrackData object
            
        Returns:
            AirDuctsData object
        """
        pass
    
    def compute_tangent_at(self, points: np.ndarray, index: int) -> np.ndarray:
        """Compute tangent vector at a point on the boundary."""
        if index == 0:
            tangent = points[1] - points[0]
        elif index == len(points) - 1:
            tangent = points[-1] - points[-2]
        else:
            tangent = points[index+1] - points[index-1]
        
        # Normalize
        norm = np.linalg.norm(tangent)
        if norm > 0:
            tangent = tangent / norm
        else:
            tangent = np.array([1.0, 0.0])
        
        return tangent
    
    def compute_obb_angle_from_tangent(self, tangent: np.ndarray) -> float:
        """Compute OBB orientation angle from tangent vector."""
        return np.arctan2(tangent[1], tangent[0])
    
    def compute_hausdorff_distance(self, obbs: List[OrientedBoundingBox], 
                                  boundary_curve: np.ndarray) -> float:
        """
        Compute Hausdorff distance between OBB chain and boundary curve.
        
        Args:
            obbs: List of placed OBBs
            boundary_curve: Nx2 array of boundary points
            
        Returns:
            Hausdorff distance
        """
        # Sample points from OBB surfaces
        obb_points = []
        for obb in obbs:
            # Sample corners and midpoints
            obb_points.extend(obb.corners)
            for i in range(4):
                mid = (obb.corners[i] + obb.corners[(i+1)%4]) / 2
                obb_points.append(mid)
        
        obb_points = np.array(obb_points)
        
        # Compute distances from OBB points to boundary
        max_dist_to_boundary = 0.0
        for p in obb_points:
            # Find closest point on boundary
            distances = np.linalg.norm(boundary_curve - p, axis=1)
            min_dist = np.min(distances)
            max_dist_to_boundary = max(max_dist_to_boundary, min_dist)
        
        # Compute distances from boundary points to OBBs
        max_dist_to_obbs = 0.0
        for p in boundary_curve:
            # Find closest point on any OBB
            min_dist = float('inf')
            for obb in obbs:
                # Distance to OBB (simplified: distance to edges)
                if obb.contains_point(p):
                    min_dist = 0.0
                    break
                else:
                    # Distance to nearest corner
                    corner_dists = np.linalg.norm(obb.corners - p, axis=1)
                    min_dist = min(min_dist, np.min(corner_dists))
            max_dist_to_obbs = max(max_dist_to_obbs, min_dist)
        
        return max(max_dist_to_boundary, max_dist_to_obbs)
    
    def optimize_obb_angles(self, obbs: List[OrientedBoundingBox], 
                           boundary_curve: np.ndarray,
                           max_iterations: int = 50) -> List[OrientedBoundingBox]:
        """
        Optimize OBB angles to minimize Hausdorff distance.
        
        Args:
            obbs: List of OBBs
            boundary_curve: Boundary curve points
            max_iterations: Maximum optimization iterations
            
        Returns:
            Optimized OBBs
        """
        # Simple gradient-free optimization (could be improved with scipy.optimize)
        for _ in range(max_iterations):
            improved = False
            for i, obb in enumerate(obbs):
                # Try small angle adjustments
                for delta in [-0.05, 0.05]:
                    original_angle = obb.angle
                    obb.angle += delta
                    
                    # Recompute corners
                    obb.corners = obb.compute_corners()
                    
                    # Compute new distance
                    new_distance = self.compute_hausdorff_distance(obbs, boundary_curve)
                    
                    if new_distance < self.gap_tolerance:
                        improved = True
                        break
                    else:
                        # Revert change
                        obb.angle = original_angle
                        obb.corners = obb.compute_corners()
                
                if improved:
                    break
        
        return obbs


class ChainAirDuctPlacer(AirDuctPlacer):
    """
    Base class for chain-based air duct placement (greedy fitting + optimization).
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.max_gap = config.get('max_gap', 0.05)  # Maximum allowed gap
        self.optimization_iters = config.get('optimization_iters', 50)
    
    @abstractmethod
    def greedy_fit(self, boundary_curve: np.ndarray) -> List[OrientedBoundingBox]:
        """
        Greedy algorithm to place OBBs along boundary curve.
        
        Args:
            boundary_curve: Nx2 array of boundary points
            
        Returns:
            List of OBBs
        """
        pass
    
    def solve_constraints(self, obbs: List[OrientedBoundingBox],
                         boundary_curve: np.ndarray) -> List[OrientedBoundingBox]:
        """
        Solve constraint satisfaction (end-to-start contact).
        
        Args:
            obbs: Initial OBBs
            boundary_curve: Boundary curve
            
        Returns:
            Adjusted OBBs
        """
        # Adjust positions to ensure contact
        for i in range(len(obbs) - 1):
            # Get end point of current OBB
            current_obb = obbs[i]
            next_obb = obbs[i + 1]
            
            # End points along centerline
            current_end = current_obb.center + np.array([current_obb.length/2 * np.cos(current_obb.angle),
                                                         current_obb.length/2 * np.sin(current_obb.angle)])
            next_start = next_obb.center - np.array([next_obb.length/2 * np.cos(next_obb.angle),
                                                    next_obb.length/2 * np.sin(next_obb.angle)])
            
            # Adjust positions to close the gap
            gap = np.linalg.norm(next_start - current_end)
            if gap > self.gap_tolerance:
                # Move next OBB to close gap
                shift = current_end - next_start
                next_obb.center = next_obb.center + shift
                next_obb.corners = next_obb.compute_corners()
        
        return obbs


class ProbabilisticAirDuctPlacer(AirDuctPlacer):
    """
    Base class for probabilistic air duct placement.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.num_ducts = config.get('num_ducts', 50)
        self.min_spacing = config.get('min_spacing', 0.5)
        self.max_spacing = config.get('max_spacing', 2.0)
        self.placement_density = config.get('placement_density', 0.3)
    
    def sample_positions_along_curve(self, curve: np.ndarray, 
                                     num_samples: int) -> List[np.ndarray]:
        """
        Sample positions along a curve with equal arc length spacing.
        
        Args:
            curve: Nx2 array of points
            num_samples: Number of positions to sample
            
        Returns:
            List of sampled positions
        """
        # Compute arc lengths
        diffs = np.diff(curve, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        cumulative_lengths = np.zeros(len(curve))
        cumulative_lengths[1:] = np.cumsum(segment_lengths)
        total_length = cumulative_lengths[-1]
        
        # Sample arc lengths
        sampled_s = np.linspace(0, total_length, num_samples)
        
        # Interpolate positions
        sampled_positions = []
        for s in sampled_s:
            # Find segment containing s
            idx = np.searchsorted(cumulative_lengths, s) - 1
            idx = max(0, min(idx, len(curve) - 2))
            
            # Interpolate between points
            t = (s - cumulative_lengths[idx]) / (segment_lengths[idx] + 1e-6)
            pos = curve[idx] + t * (curve[idx + 1] - curve[idx])
            sampled_positions.append(pos)
        
        return sampled_positions
    
    def compute_probability_weights(self, track: Any, positions: List[np.ndarray]) -> np.ndarray:
        """
        Compute probability weights for positions based on track features.
        
        Args:
            track: TrackData object
            positions: List of candidate positions
            
        Returns:
            Array of weights (higher = more likely to place duct)
        """
        # Default: uniform weights
        # Subclasses can override to use curvature, width, etc.
        return np.ones(len(positions))