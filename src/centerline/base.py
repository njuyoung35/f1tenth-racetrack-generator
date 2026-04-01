"""
Base classes and data structures for centerline generation.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Any, Optional, List, Tuple, Callable
import numpy as np
from enum import Enum


class CenterlineRepresentation(Enum):
    """Enum for centerline representation types."""
    POINTS = "points"
    SPLINE = "spline"
    FRENET = "frenet"


@dataclass
class CenterlineData:
    """
    Data class for storing centerline information.
    
    Attributes:
        points: Nx2 array of (x, y) coordinates
        curvature: N array of curvature values (kappa)
        s: N array of arc length (cumulative distance)
        tangent: Nx2 array of tangent vectors (optional)
        normal: Nx2 array of normal vectors (optional)
        control_points: For spline representation (optional)
        representation: Type of representation used
        metadata: Additional metadata (algorithm name, parameters, etc.)
    """
    points: np.ndarray  # Nx2
    curvature: np.ndarray  # N
    s: np.ndarray  # N
    tangent: Optional[np.ndarray] = None  # Nx2
    normal: Optional[np.ndarray] = None  # Nx2
    control_points: Optional[np.ndarray] = None  # Mx2 for spline
    representation: CenterlineRepresentation = CenterlineRepresentation.POINTS
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate data after initialization."""
        if self.points is not None:
            n_points = len(self.points)
            if self.curvature is not None and len(self.curvature) != n_points:
                raise ValueError(f"curvature length {len(self.curvature)} != points length {n_points}")
            if self.s is not None and len(self.s) != n_points:
                raise ValueError(f"s length {len(self.s)} != points length {n_points}")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            'points': self.points.tolist(),
            'curvature': self.curvature.tolist(),
            's': self.s.tolist(),
            'tangent': self.tangent.tolist() if self.tangent is not None else None,
            'normal': self.normal.tolist() if self.normal is not None else None,
            'control_points': self.control_points.tolist() if self.control_points is not None else None,
            'representation': self.representation.value,
            'metadata': self.metadata
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CenterlineData':
        """Create from dictionary."""
        return cls(
            points=np.array(data['points']),
            curvature=np.array(data['curvature']),
            s=np.array(data['s']),
            tangent=np.array(data['tangent']) if data.get('tangent') else None,
            normal=np.array(data['normal']) if data.get('normal') else None,
            control_points=np.array(data['control_points']) if data.get('control_points') else None,
            representation=CenterlineRepresentation(data['representation']),
            metadata=data.get('metadata', {})
        )


class CenterlineGenerator(ABC):
    """
    Abstract base class for centerline generation algorithms.
    
    All centerline generators must implement the generate() method.
    They can optionally implement additional methods for optimization,
    preference learning, or interactive generation.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        """
        Initialize the centerline generator.
        
        Args:
            config: Configuration dictionary with parameters
            seed: Random seed for reproducibility
        """
        self.config = config
        self.seed = seed
        if seed is not None:
            np.random.seed(seed)
        
        # Extract common parameters
        self.width = config.get('width', 100.0)
        self.height = config.get('height', 100.0)
        self.num_points = config.get('num_points', 200)
        self.max_curvature = config.get('max_curvature', 0.5)
        
    @abstractmethod
    def generate(self) -> CenterlineData:
        """
        Generate a centerline.
        
        Returns:
            CenterlineData object containing the generated centerline
        """
        pass
    
    def compute_curvature(self, points: np.ndarray, method: str = 'finite_difference') -> np.ndarray:
        """
        Compute curvature from points (utility method).
        
        Args:
            points: Nx2 array of points
            method: Method to compute curvature ('finite_difference' or 'circle_fitting')
            
        Returns:
            N array of curvature values
        """
        if method == 'finite_difference':
            return self._curvature_finite_difference(points)
        elif method == 'circle_fitting':
            return self._curvature_circle_fitting(points)
        else:
            raise ValueError(f"Unknown curvature method: {method}")
    
    def _curvature_finite_difference(self, points: np.ndarray) -> np.ndarray:
        """Compute curvature using finite differences."""
        # First derivatives
        dx = np.gradient(points[:, 0])
        dy = np.gradient(points[:, 1])
        
        # Second derivatives
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Curvature formula: |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        numerator = np.abs(dx * ddy - dy * ddx)
        denominator = (dx**2 + dy**2)**(1.5)
        
        # Avoid division by zero
        curvature = np.zeros_like(numerator)
        valid = denominator > 1e-6
        curvature[valid] = numerator[valid] / denominator[valid]
        
        return curvature
    
    def _curvature_circle_fitting(self, points: np.ndarray) -> np.ndarray:
        """
        Compute curvature by fitting circles to consecutive triplets.
        More robust for noisy data.
        """
        n = len(points)
        curvature = np.zeros(n)
        
        for i in range(1, n-1):
            p1 = points[i-1]
            p2 = points[i]
            p3 = points[i+1]
            
            # Compute circumradius
            a = np.linalg.norm(p2 - p3)
            b = np.linalg.norm(p1 - p3)
            c = np.linalg.norm(p1 - p2)
            
            if a * b * c > 0:
                # Circumradius formula
                area = 0.5 * abs(np.cross(p2 - p1, p3 - p1))
                radius = (a * b * c) / (4 * area + 1e-6)
                curvature[i] = 1.0 / radius
                
                # Sign: positive for left turns, negative for right turns
                # Using cross product to determine orientation
                cross = np.cross(p2 - p1, p3 - p2)
                curvature[i] *= np.sign(cross)
        
        # Handle boundaries
        curvature[0] = curvature[1]
        curvature[-1] = curvature[-2]
        
        return curvature
    
    def compute_arc_length(self, points: np.ndarray) -> np.ndarray:
        """Compute cumulative arc length from points."""
        diffs = np.diff(points, axis=0)
        segment_lengths = np.linalg.norm(diffs, axis=1)
        s = np.zeros(len(points))
        s[1:] = np.cumsum(segment_lengths)
        return s
    
    def enforce_max_curvature(self, centerline: CenterlineData) -> CenterlineData:
        """
        Enforce maximum curvature constraint by smoothing or scaling.
        Can be overridden by subclasses.
        """
        if self.max_curvature is None:
            return centerline
            
        # Simple clamping - more sophisticated methods can be implemented in subclasses
        centerline.curvature = np.clip(centerline.curvature, -self.max_curvature, self.max_curvature)
        return centerline
    
    def get_parameter_bounds(self) -> Dict[str, Tuple[float, float]]:
        """
        Get parameter bounds for optimization (to be implemented by subclasses).
        Returns a dictionary mapping parameter names to (min, max) tuples.
        """
        return {}


class InteractiveCenterlineGenerator(CenterlineGenerator):
    """
    Abstract base class for interactive generators that can incorporate
    human feedback (preference learning, pairwise comparisons).
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.feedback_history = []
    
    @abstractmethod
    def generate_candidates(self, num_candidates: int = 2) -> List[CenterlineData]:
        """
        Generate multiple candidate centerlines for human evaluation.
        
        Args:
            num_candidates: Number of candidates to generate
            
        Returns:
            List of CenterlineData objects
        """
        pass
    
    @abstractmethod
    def update_with_feedback(self, selected_index: int, candidates: List[CenterlineData]):
        """
        Update generator based on human feedback.
        
        Args:
            selected_index: Index of the selected candidate
            candidates: List of candidates presented
        """
        pass
    
    def record_feedback(self, selected_index: int, candidates: List[CenterlineData]):
        """Record feedback for analysis."""
        self.feedback_history.append({
            'selected_index': selected_index,
            'candidates': [c.to_dict() for c in candidates],
            'timestamp': None  # Can add datetime here
        })


class OptimizableCenterlineGenerator(CenterlineGenerator):
    """
    Abstract base class for centerline generators that use optimization.
    Provides hooks for objective function evaluation.
    """
    
    def __init__(self, config: Dict[str, Any], seed: Optional[int] = None):
        super().__init__(config, seed)
        self.objective_weights = config.get('objective_weights', {
            'curvature_entropy': 1.0,
            'tortuosity': 0.5,
            'self_avoidance': 100.0,
            'length_penalty': 0.1
        })
    
    @abstractmethod
    def compute_objective(self, centerline: CenterlineData) -> float:
        """
        Compute objective function value for a centerline.
        Lower is better (minimization).
        """
        pass
    
    def compute_curvature_entropy(self, curvature: np.ndarray, num_bins: int = 20) -> float:
        """Compute entropy of curvature distribution."""
        hist, _ = np.histogram(np.abs(curvature), bins=num_bins, density=True)
        hist = hist[hist > 0]  # Remove zero entries
        entropy = -np.sum(hist * np.log(hist + 1e-10))
        return entropy
    
    def compute_tortuosity(self, points: np.ndarray) -> float:
        """Compute tortuosity = arc length / straight-line distance."""
        arc_length = self.compute_arc_length(points)[-1]
        straight_distance = np.linalg.norm(points[-1] - points[0])
        return arc_length / (straight_distance + 1e-6)
    
    def compute_self_avoidance_penalty(self, points: np.ndarray, margin: float = 0.5) -> float:
        """
        Compute penalty for self-intersections.
        Higher penalty for closer self-intersections.
        """
        penalty = 0.0
        n = len(points)
        
        # Check for intersections between non-adjacent segments
        for i in range(n - 2):
            for j in range(i + 2, n - 1):
                # Skip adjacent segments
                if j == i + 1:
                    continue
                    
                p1 = points[i]
                p2 = points[i + 1]
                p3 = points[j]
                p4 = points[j + 1]
                
                # Check if segments intersect
                if self._segments_intersect(p1, p2, p3, p4):
                    # Compute intersection distance
                    intersection = self._line_intersection(p1, p2, p3, p4)
                    if intersection is not None:
                        dist_to_boundary = min(
                            self._point_segment_distance(intersection, points[i-1], p1) if i > 0 else np.inf,
                            self._point_segment_distance(intersection, p2, points[i+2]) if i+2 < n else np.inf
                        )
                        penalty += max(0, margin - dist_to_boundary)
        
        return penalty
    
    def _segments_intersect(self, p1, p2, p3, p4) -> bool:
        """Check if two line segments intersect."""
        def orientation(pa, pb, pc):
            val = (pb[1] - pa[1]) * (pc[0] - pb[0]) - (pb[0] - pa[0]) * (pc[1] - pb[1])
            if abs(val) < 1e-6:
                return 0
            return 1 if val > 0 else -1
        
        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)
        
        if o1 != o2 and o3 != o4:
            return True
        
        # Check collinear cases
        if o1 == 0 and self._on_segment(p1, p3, p2):
            return True
        if o2 == 0 and self._on_segment(p1, p4, p2):
            return True
        if o3 == 0 and self._on_segment(p3, p1, p4):
            return True
        if o4 == 0 and self._on_segment(p3, p2, p4):
            return True
        
        return False
    
    def _on_segment(self, p, q, r):
        """Check if point q lies on segment pr."""
        return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
                min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))
    
    def _line_intersection(self, p1, p2, p3, p4):
        """Find intersection of two lines."""
        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]
        
        xdiff = (p1[0] - p2[0], p3[0] - p4[0])
        ydiff = (p1[1] - p2[1], p3[1] - p4[1])
        
        div = det(xdiff, ydiff)
        if abs(div) < 1e-6:
            return None
            
        d = (det(p1, p2), det(p3, p4))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return np.array([x, y])
    
    def _point_segment_distance(self, p, a, b):
        """Compute distance from point p to segment ab."""
        ab = b - a
        ap = p - a
        t = np.dot(ap, ab) / (np.dot(ab, ab) + 1e-6)
        t = np.clip(t, 0, 1)
        projection = a + t * ab
        return np.linalg.norm(p - projection)