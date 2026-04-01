"""
Base classes and data structures for track (wall) generation.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, Tuple, List
import numpy as np
from enum import Enum


class TrackRepresentation(Enum):
    """Enum for track representation types."""
    BOUNDARY_POINTS = "boundary_points"
    SIGNED_DISTANCE = "signed_distance"
    PROBABILISTIC = "probabilistic"
    OCCUPANCY_GRID = "occupancy_grid"


@dataclass
class TrackData:
    """
    Data class for storing track information.
    
    Attributes:
        centerline: CenterlineData from previous step
        left_boundary: Nx2 array of left boundary points
        right_boundary: Nx2 array of right boundary points
        width_profile: N array of track width at each centerline point
        curvature_coupled_width: Whether width is coupled with curvature
        signed_distance_field: Optional signed distance function
        representation: Type of representation used
        metadata: Additional metadata
    """
    centerline: Any  # CenterlineData (avoid circular import)
    left_boundary: np.ndarray  # Nx2
    right_boundary: np.ndarray  # Nx2
    width_profile: np.ndarray  # N
    curvature_coupled_width: bool = False
    signed_distance_field: Optional[np.ndarray] = None
    representation: TrackRepresentation = TrackRepresentation.BOUNDARY_POINTS
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate data."""
        n = len(self.left_boundary)
        if len(self.right_boundary) != n:
            raise ValueError(f"left_boundary length {n} != right_boundary length {len(self.right_boundary)}")
        if len(self.width_profile) != n:
            raise ValueError(f"width_profile length {len(self.width_profile)} != {n}")
    
    def get_width_at(self, s_index: int) -> float:
        """Get track width at a specific arc length index."""
        return self.width_profile[s_index]
    
    def get_boundary_at(self, s_index: int, side: str = 'left') -> np.ndarray:
        """Get boundary point at a specific index."""
        if side == 'left':
            return self.left_boundary[s_index]
        elif side == 'right':
            return self.right_boundary[s_index]
        else:
            raise ValueError(f"Unknown side: {side}")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'centerline': self.centerline.to_dict() if hasattr(self.centerline, 'to_dict') else self.centerline,
            'left_boundary': self.left_boundary.tolist(),
            'right_boundary': self.right_boundary.tolist(),
            'width_profile': self.width_profile.tolist(),
            'curvature_coupled_width': self.curvature_coupled_width,
            'representation': self.representation.value,
            'metadata': self.metadata
        }


class TrackGenerator(ABC):
    """
    Abstract base class for track generation algorithms.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        """
        Initialize track generator.
        
        Args:
            config: Configuration dictionary
            seed: Random seed
        """
        self.config = config
        self.seed = seed
        if seed is not None:
            np.random.seed(seed)
        
        # Common parameters
        self.base_width = config.get('base_width', 0.5)
        self.min_width = config.get('min_width', 0.3)
        self.max_width = config.get('max_width', 1.0)
    
    @abstractmethod
    def generate(self, centerline: Any) -> TrackData:
        """
        Generate track boundaries from centerline.
        
        Args:
            centerline: CenterlineData object
            
        Returns:
            TrackData object
        """
        pass
    
    def offset_curve(self, points: np.ndarray, distance: float, 
                     direction: str = 'left') -> np.ndarray:
        """
        Offset a curve by a given distance perpendicular to the curve.
        
        Args:
            points: Nx2 array of points
            distance: Offset distance (positive)
            direction: 'left' or 'right'
            
        Returns:
            Nx2 array of offset points
        """
        n = len(points)
        offset_points = np.zeros_like(points)
        
        for i in range(n):
            # Get tangent and normal at point i
            if i == 0:
                # Forward difference for first point
                tangent = points[1] - points[0]
            elif i == n - 1:
                # Backward difference for last point
                tangent = points[-1] - points[-2]
            else:
                # Central difference
                tangent = points[i+1] - points[i-1]
            
            # Normalize tangent
            tangent = tangent / (np.linalg.norm(tangent) + 1e-6)
            
            # Normal vector (perpendicular)
            normal = np.array([-tangent[1], tangent[0]])
            
            # Flip sign for right offset
            if direction == 'right':
                normal = -normal
            
            # Apply offset
            offset_points[i] = points[i] + distance * normal
        
        return offset_points
    
    def smooth_boundary(self, boundary: np.ndarray, sigma: float = 1.0) -> np.ndarray:
        """
        Smooth boundary points using Gaussian filter.
        
        Args:
            boundary: Nx2 array of points
            sigma: Smoothing parameter
            
        Returns:
            Smoothed boundary
        """
        from scipy.ndimage import gaussian_filter1d
        
        smoothed = np.zeros_like(boundary)
        smoothed[:, 0] = gaussian_filter1d(boundary[:, 0], sigma, mode='wrap')
        smoothed[:, 1] = gaussian_filter1d(boundary[:, 1], sigma, mode='wrap')
        return smoothed
    
    def enforce_min_width(self, track: TrackData) -> TrackData:
        """
        Enforce minimum width constraint.
        """
        # Ensure width is at least min_width
        track.width_profile = np.maximum(track.width_profile, self.min_width)
        
        # Recompute boundaries if needed
        # This is a simplified version - subclasses may need more sophisticated handling
        half_widths = track.width_profile / 2.0
        
        # Recompute boundaries
        centerline_points = track.centerline.points
        track.left_boundary = self.offset_curve(centerline_points, half_widths, 'left')
        track.right_boundary = self.offset_curve(centerline_points, half_widths, 'right')
        
        return track


class StochasticTrackGenerator(TrackGenerator):
    """
    Base class for track generators with stochastic components.
    Adds methods for Perlin noise and other stochastic processes.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.noise_amplitude = config.get('noise_amplitude', 0.05)
        self.noise_scale = config.get('noise_scale', 5.0)
    
    def generate_perlin_noise_1d(self, n_points: int) -> np.ndarray:
        """
        Generate 1D Perlin noise for track width variation.
        
        Args:
            n_points: Number of points
            
        Returns:
            Noise array of length n_points
        """
        # Simple implementation using sine waves with random phases
        # For full Perlin noise, consider using the 'noise' library
        np.random.seed(self.seed)
        
        t = np.linspace(0, self.noise_scale * np.pi, n_points)
        noise = np.zeros(n_points)
        
        # Add multiple frequencies
        for freq in [1, 2, 4, 8]:
            phase = np.random.uniform(0, 2 * np.pi)
            amplitude = self.noise_amplitude / freq
            noise += amplitude * np.sin(freq * t + phase)
        
        return noise


class CurvatureCoupledTrackGenerator(TrackGenerator):
    """
    Base class for track generators where width is coupled with curvature.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.alpha = config.get('alpha', 0.2)  # Width variation factor
        self.beta = config.get('beta', 1.0)    # Curvature sensitivity
    
    def compute_width_from_curvature(self, curvature: np.ndarray) -> np.ndarray:
        """
        Compute width profile based on curvature.
        
        Formula: w(s) = base_width + alpha / (1 + beta * |kappa(s)|)
        """
        # Avoid division by zero
        denominator = 1.0 + self.beta * np.abs(curvature)
        width = self.base_width + self.alpha / denominator
        
        # Clip to allowed range
        width = np.clip(width, self.min_width, self.max_width)
        
        return width