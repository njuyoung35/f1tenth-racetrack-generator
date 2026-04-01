"""
Air duct placement using a chain of oriented bounding boxes (OBBs) along the track boundary.
Greedy fitting, constraint solving, and optimization to minimize Hausdorff distance.
"""
import numpy as np
from scipy.interpolate import interp1d
from typing import List, Optional, Tuple
from .base import AirDuctPlacer, OrientedBoundingBox, AirDuctsData, ChainAirDuctPlacer


class OBBChainAirDuctPlacer(ChainAirDuctPlacer):
    """
    Places air ducts as a chain of OBBs along the left boundary of the track.
    """
    
    def __init__(self, config: dict, seed: Optional[int] = None):
        super().__init__(config, seed)
        self.side = config.get('side', 'left')  # 'left' or 'right'
        self.sample_resolution = config.get('sample_resolution', 0.05)  # meters
        self.interpolation_kind = config.get('interpolation_kind', 'cubic')
    
    def place(self, track) -> AirDuctsData:
        """
        Place air ducts along the specified boundary.
        """
        # Get the boundary curve
        if self.side == 'left':
            boundary = track.left_boundary
        else:
            boundary = track.right_boundary
        
        # Interpolate boundary for continuous sampling
        interp_func = self._interpolate_boundary(boundary)
        
        # Greedy fit: place OBBs along the curve
        obbs = self.greedy_fit(interp_func)
        
        # Solve constraints: ensure end-to-start contact
        obbs = self.solve_constraints(obbs, interp_func)
        
        # Optimize angles to minimize Hausdorff distance
        obbs = self.optimize_obb_angles(obbs, boundary, self.optimization_iters)
        
        # Create AirDuctsData
        air_ducts = AirDuctsData(
            obbs=obbs,
            metadata={
                'side': self.side,
                'duct_length': self.duct_length,
                'duct_width': self.duct_width,
                'num_ducts': len(obbs)
            }
        )
        
        return air_ducts
    
    def _interpolate_boundary(self, boundary: np.ndarray):
        """
        Create an interpolation function for the boundary curve.
        Returns a function f(t) where t in [0,1] gives (x,y).
        """
        # Parameterize by arc length
        s = np.zeros(len(boundary))
        for i in range(1, len(boundary)):
            s[i] = s[i-1] + np.linalg.norm(boundary[i] - boundary[i-1])
        t = s / s[-1]
        
        # Interpolate x and y as functions of t
        fx = interp1d(t, boundary[:,0], kind=self.interpolation_kind, 
                      bounds_error=False, fill_value='extrapolate')
        fy = interp1d(t, boundary[:,1], kind=self.interpolation_kind,
                      bounds_error=False, fill_value='extrapolate')
        
        return lambda u: np.array([fx(u), fy(u)])
    
    def greedy_fit(self, curve_func) -> List[OrientedBoundingBox]:
        """
        Greedy algorithm to place OBBs along the curve.
        Starting at t=0, place a duct oriented along the tangent, then step forward
        by duct length, repeat until t >= 1.
        """
        obbs = []
        t = 0.0
        while t < 1.0:
            # Get position and tangent at current t
            pos = curve_func(t)
            tangent = self._compute_tangent(curve_func, t)
            angle = self.compute_obb_angle_from_tangent(tangent)
            
            # Create OBB
            obb = OrientedBoundingBox(
                center=pos,
                length=self.duct_length,
                width=self.duct_width,
                angle=angle
            )
            obbs.append(obb)
            
            # Move forward along the curve by duct length (in parameter space)
            # Approximate: step in t such that arc length increment ≈ duct_length
            # For simplicity, step by fixed delta_t (could be improved)
            # We'll compute using a small step and adjust.
            # For this prototype, we'll assume uniform sampling.
            # More accurate: integrate arc length.
            # Since we have the curve function, we can sample points along the curve.
            # Let's use a simple approach: sample points at small dt, compute cumulative distance.
            step = 0.01  # initial guess
            target_s = self.duct_length
            s_current = 0
            t_new = t
            while s_current < target_s and t_new < 1.0:
                t_new += step
                pos_new = curve_func(t_new)
                s_current += np.linalg.norm(pos_new - pos)
                if t_new >= 1.0:
                    break
            t = t_new
        
        return obbs
    
    def _compute_tangent(self, curve_func, t: float, delta: float = 0.001) -> np.ndarray:
        """Compute tangent vector at parameter t."""
        pos = curve_func(t)
        pos_plus = curve_func(min(t + delta, 1.0))
        tangent = pos_plus - pos
        norm = np.linalg.norm(tangent)
        if norm > 0:
            tangent = tangent / norm
        else:
            tangent = np.array([1.0, 0.0])
        return tangent
    
    def solve_constraints(self, obbs: List[OrientedBoundingBox], curve_func) -> List[OrientedBoundingBox]:
        """
        Adjust OBB positions so that the end of one touches the start of the next.
        We'll use a simple iterative adjustment.
        """
        if len(obbs) <= 1:
            return obbs
        
        for i in range(len(obbs) - 1):
            current = obbs[i]
            next_obb = obbs[i + 1]
            
            # Compute end point of current OBB (center + half-length in direction of angle)
            end_current = current.center + current.length/2 * np.array([np.cos(current.angle), np.sin(current.angle)])
            # Compute start point of next OBB (center - half-length in direction of angle)
            start_next = next_obb.center - next_obb.length/2 * np.array([np.cos(next_obb.angle), np.sin(next_obb.angle)])
            
            # Compute gap
            gap = np.linalg.norm(start_next - end_current)
            if gap > self.gap_tolerance:
                # Move next OBB to close the gap
                shift = end_current - start_next
                next_obb.center = next_obb.center + shift
                next_obb.corners = next_obb.compute_corners()
        
        return obbs
    
    def optimize_obb_angles(self, obbs: List[OrientedBoundingBox], 
                           boundary: np.ndarray,
                           max_iterations: int) -> List[OrientedBoundingBox]:
        """
        Simple gradient-free angle optimization.
        For each OBB, try small angle adjustments to minimize Hausdorff distance.
        """
        # Convert boundary to continuous function for distance computation
        # (Reuse interpolation function if needed)
        # For simplicity, we'll use the raw boundary points for distance calculation.
        # But Hausdorff distance requires many samples.
        
        # We'll use a simplified metric: sum of distances of OBB corners to boundary.
        for _ in range(max_iterations):
            improved = False
            for i, obb in enumerate(obbs):
                best_distance = self._compute_obb_to_boundary_distance(obb, boundary)
                best_angle = obb.angle
                for delta in [-0.05, 0.05]:
                    obb.angle += delta
                    obb.corners = obb.compute_corners()
                    new_distance = self._compute_obb_to_boundary_distance(obb, boundary)
                    if new_distance < best_distance:
                        best_distance = new_distance
                        best_angle = obb.angle
                        improved = True
                    else:
                        obb.angle -= delta  # revert
                        obb.corners = obb.compute_corners()
                obb.angle = best_angle
                obb.corners = obb.compute_corners()
            if not improved:
                break
        
        return obbs
    
    def _compute_obb_to_boundary_distance(self, obb: OrientedBoundingBox, boundary: np.ndarray) -> float:
        """Compute maximum distance from OBB corners to boundary."""
        corners = obb.corners
        # For each corner, find closest point on boundary
        max_dist = 0.0
        for corner in corners:
            dists = np.linalg.norm(boundary - corner, axis=1)
            min_dist = np.min(dists)
            if min_dist > max_dist:
                max_dist = min_dist
        return max_dist