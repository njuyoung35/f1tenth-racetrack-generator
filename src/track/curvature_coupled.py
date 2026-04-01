"""
Track generation where width varies with curvature:
w(s) = w_base + alpha / (1 + beta * |kappa(s)|)
"""
import numpy as np
from typing import Optional
from .base import TrackGenerator, TrackData, CurvatureCoupledTrackGenerator


class CurvatureCoupledTrackGenerator(CurvatureCoupledTrackGenerator):
    """
    Generate track boundaries based on curvature-dependent width.
    """
    
    def generate(self, centerline) -> TrackData:
        """
        Compute width profile from centerline curvature and offset boundaries.
        """
        # Get curvature (absolute value for widening effect)
        curvature = np.abs(centerline.curvature)
        
        # Compute width profile
        width = self.compute_width_from_curvature(curvature)
        
        # Offset left and right boundaries
        half_width = width / 2.0
        left_boundary = self.offset_curve(centerline.points, half_width, 'left')
        right_boundary = self.offset_curve(centerline.points, half_width, 'right')
        
        # Optional smoothing
        if self.config.get('smooth_boundary', False):
            sigma = self.config.get('smoothing_sigma', 1.0)
            left_boundary = self.smooth_boundary(left_boundary, sigma)
            right_boundary = self.smooth_boundary(right_boundary, sigma)
        
        # Create TrackData
        track = TrackData(
            centerline=centerline,
            left_boundary=left_boundary,
            right_boundary=right_boundary,
            width_profile=width,
            curvature_coupled_width=True,
            metadata={
                'alpha': self.alpha,
                'beta': self.beta,
                'base_width': self.base_width
            }
        )
        
        # Enforce minimum width if needed
        if self.min_width is not None:
            track = self.enforce_min_width(track)
        
        return track