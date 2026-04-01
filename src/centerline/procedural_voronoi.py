"""
Centerline generation using Voronoi diagram of random points.
Based on the approach from https://github.com/mvanlobensels/random-track-generator
"""
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d
from scipy.interpolate import splprep, splev
from typing import List, Tuple, Optional
from .base import CenterlineGenerator, CenterlineData


class VoronoiCenterlineGenerator(CenterlineGenerator):
    """
    Generates a closed loop centerline by extracting a path from a Voronoi diagram.
    """
    
    def __init__(self, config: dict, seed: Optional[int] = None):
        super().__init__(config, seed)
        self.num_voronoi_points = config.get('num_voronoi_points', 50)
        self.boundary_margin = config.get('boundary_margin', 5.0)  # Keep points away from edges
        self.spline_smoothing = config.get('spline_smoothing', 0.0)  # s parameter for splprep
    
    def generate(self) -> CenterlineData:
        """Generate centerline using Voronoi method."""
        # 1. Generate random points within the region
        points = self._generate_random_points()
        
        # 2. Compute Voronoi diagram
        vor = Voronoi(points)
        
        # 3. Extract a closed loop from Voronoi edges
        loop_points = self._extract_voronoi_loop(vor)
        
        # 4. Smooth and resample the loop
        smoothed_points = self._smooth_loop(loop_points)
        
        # 5. Compute curvature and arc length
        curvature = self.compute_curvature(smoothed_points)
        s = self.compute_arc_length(smoothed_points)
        
        # 6. Enforce maximum curvature if specified
        centerline = CenterlineData(
            points=smoothed_points,
            curvature=curvature,
            s=s,
            representation=self.config.get('representation', 'points'),
            metadata={'algorithm': 'voronoi', 'num_points': self.num_voronoi_points}
        )
        
        if self.max_curvature is not None:
            centerline = self.enforce_max_curvature(centerline)
        
        return centerline
    
    def _generate_random_points(self) -> np.ndarray:
        """Generate random points inside bounding box, avoiding margins."""
        x_min = self.boundary_margin
        x_max = self.width - self.boundary_margin
        y_min = self.boundary_margin
        y_max = self.height - self.boundary_margin
        
        points = np.random.uniform(
            low=[x_min, y_min],
            high=[x_max, y_max],
            size=(self.num_voronoi_points, 2)
        )
        return points
    
    def _extract_voronoi_loop(self, vor: Voronoi) -> np.ndarray:
        """
        Extract a closed loop from the Voronoi edges.
        This is a simplified version: we take the largest connected component
        of edges and then close it. A more sophisticated method would follow
        the path that stays within the bounding box.
        """
        # Build graph from ridge vertices
        edges = []
        for ridge in vor.ridge_vertices:
            if -1 not in ridge and all(v < len(vor.vertices) for v in ridge):
                edges.append(ridge)
        
        # Simple approach: pick a starting vertex and traverse edges
        if not edges:
            raise RuntimeError("No valid Voronoi edges found")
        
        # Build adjacency list
        adj = {}
        for v1, v2 in edges:
            adj.setdefault(v1, []).append(v2)
            adj.setdefault(v2, []).append(v1)
        
        # Find a vertex with degree 2 (likely part of a loop)
        start = None
        for v, neighbors in adj.items():
            if len(neighbors) == 2:
                start = v
                break
        if start is None:
            start = list(adj.keys())[0]  # fallback
        
        # Traverse to form a closed loop
        path = [start]
        prev = None
        current = start
        while True:
            next_vertices = adj[current]
            if len(next_vertices) == 2:
                nxt = next_vertices[0] if next_vertices[0] != prev else next_vertices[1]
            else:
                nxt = next_vertices[0]  # degree 1, dead end, but we'll continue
            if nxt == start:
                break
            path.append(nxt)
            prev, current = current, nxt
        
        # Convert vertex indices to coordinates
        points = vor.vertices[path]
        
        # Close the loop
        points = np.vstack([points, points[0]])
        
        return points
    
    def _smooth_loop(self, points: np.ndarray) -> np.ndarray:
        """
        Smooth the loop using B-spline interpolation and resample to self.num_points.
        """
        # Remove duplicate start/end if present
        if np.allclose(points[0], points[-1]):
            points = points[:-1]
        
        # For closed curve, we need periodic spline
        try:
            # Periodic spline requires parameter t to be 0..1
            t = np.linspace(0, 1, len(points))
            # Spline with periodic condition
            k = 3  # cubic spline
            tck, u = splprep([points[:,0], points[:,1]], u=t, s=self.spline_smoothing, per=True)
            # Evaluate at self.num_points
            u_new = np.linspace(0, 1, self.num_points)
            x_new, y_new = splev(u_new, tck)
            smoothed = np.column_stack([x_new, y_new])
        except Exception as e:
            # Fallback: simple linear interpolation
            print(f"Spline smoothing failed: {e}, using linear interpolation")
            # Simple linear interpolation along points
            arc = self.compute_arc_length(points)
            t = arc / arc[-1]
            t_new = np.linspace(0, 1, self.num_points)
            x_new = np.interp(t_new, t, points[:,0])
            y_new = np.interp(t_new, t, points[:,1])
            smoothed = np.column_stack([x_new, y_new])
        
        return smoothed