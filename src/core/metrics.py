"""
Comprehensive metrics computation for F1Tenth tracks.

This module provides functions to compute various geometric, functional,
and meta-properties of generated tracks for evaluation and comparison.
"""
import numpy as np
from typing import Dict, Any, List, Tuple, Optional
from dataclasses import dataclass
from scipy import stats
from scipy.spatial import KDTree
from scipy.ndimage import convolve


@dataclass
class TrackMetrics:
    """Container for all computed metrics."""
    # Geometric metrics
    total_length: float
    track_area: float
    avg_width: float
    width_variance: float
    min_width: float
    max_width: float
    
    # Curvature metrics
    avg_curvature: float
    max_curvature: float
    curvature_entropy: float
    curvature_variance: float
    curvature_skewness: float
    
    # Tortuosity and complexity
    tortuosity: float
    sinuosity: float
    num_curves: int
    curve_density: float
    
    # Track difficulty metrics
    technical_score: float
    avg_speed_estimate: float  # Estimated average speed (m/s)
    
    # Safety and racing metrics
    overtaking_opportunities: int
    overtaking_length_ratio: float
    recovery_slack: float
    lidar_feature_density: float
    
    # Self-avoidance
    self_intersection_penalty: float
    min_distance_to_self: float
    
    # Additional metadata
    metadata: Dict[str, Any]


def compute_all_metrics(exp_config: Dict[str, Any], 
                       centerline: Any,
                       track: Any, 
                       air_ducts: Any) -> Dict[str, Any]:
    """
    Compute all metrics for the generated track.
    
    Args:
        exp_config: Experiment configuration
        centerline: CenterlineData object
        track: TrackData object
        air_ducts: AirDuctsData object
        
    Returns:
        Dictionary of all computed metrics
    """
    metrics = {}
    
    # Basic track properties
    metrics.update(_compute_geometric_metrics(centerline, track))
    metrics.update(_compute_curvature_metrics(centerline))
    metrics.update(_compute_tortuosity_metrics(centerline))
    metrics.update(_compute_track_quality_metrics(track))
    metrics.update(_compute_racing_metrics(centerline, track, air_ducts))
    metrics.update(_compute_safety_metrics(centerline, track, air_ducts))
    metrics.update(_compute_self_avoidance_metrics(centerline, track))
    
    # Add metadata
    metrics['metadata'] = {
        'algorithm': exp_config.get('centerline', {}).get('type', 'unknown'),
        'track_generator': exp_config.get('track', {}).get('type', 'unknown'),
        'air_duct_placer': exp_config.get('air_duct', {}).get('type', 'unknown'),
    }
    
    return metrics


def _compute_geometric_metrics(centerline: Any, track: Any) -> Dict[str, float]:
    """Compute basic geometric properties."""
    # Total length
    total_length = centerline.s[-1] if len(centerline.s) > 0 else 0.0
    
    # Track area (approximate using polygon area between boundaries)
    left_boundary = track.left_boundary
    right_boundary = track.right_boundary
    # Create polygon by concatenating left boundary and reversed right boundary
    polygon = np.vstack([left_boundary, right_boundary[::-1]])
    track_area = 0.5 * abs(np.sum(polygon[:-1, 0] * polygon[1:, 1] - polygon[1:, 0] * polygon[:-1, 1]))
    
    # Width statistics
    width_profile = track.width_profile
    avg_width = np.mean(width_profile)
    width_variance = np.var(width_profile)
    min_width = np.min(width_profile)
    max_width = np.max(width_profile)
    
    return {
        'total_length': total_length,
        'track_area': track_area,
        'avg_width': avg_width,
        'width_variance': width_variance,
        'min_width': min_width,
        'max_width': max_width,
    }


def _compute_curvature_metrics(centerline: Any) -> Dict[str, float]:
    """Compute curvature-based metrics."""
    curvature = np.abs(centerline.curvature)
    
    avg_curvature = np.mean(curvature)
    max_curvature = np.max(curvature)
    curvature_variance = np.var(curvature)
    
    # Compute curvature entropy
    hist, _ = np.histogram(curvature, bins=20, density=True)
    hist = hist[hist > 0]
    curvature_entropy = -np.sum(hist * np.log(hist + 1e-10))
    
    # Compute skewness
    curvature_skewness = stats.skew(curvature)
    
    return {
        'avg_curvature': avg_curvature,
        'max_curvature': max_curvature,
        'curvature_entropy': curvature_entropy,
        'curvature_variance': curvature_variance,
        'curvature_skewness': curvature_skewness,
    }


def _compute_tortuosity_metrics(centerline: Any) -> Dict[str, float]:
    """Compute path complexity metrics."""
    points = centerline.points
    total_length = centerline.s[-1]
    
    # Straight line distance between start and end
    straight_distance = np.linalg.norm(points[-1] - points[0])
    
    # Tortuosity = path length / straight-line distance
    tortuosity = total_length / (straight_distance + 1e-6)
    
    # Sinuosity = 1 - (straight_distance / total_length)
    sinuosity = 1.0 - (straight_distance / (total_length + 1e-6))
    
    # Count curves (local maxima in curvature)
    curvature = centerline.curvature
    # Smooth curvature slightly to reduce noise
    from scipy.signal import find_peaks
    peaks, _ = find_peaks(np.abs(curvature), distance=10, prominence=0.05)
    num_curves = len(peaks)
    curve_density = num_curves / (total_length + 1e-6)
    
    return {
        'tortuosity': tortuosity,
        'sinuosity': sinuosity,
        'num_curves': num_curves,
        'curve_density': curve_density,
    }


def _compute_track_quality_metrics(track: Any) -> Dict[str, float]:
    """Compute track quality and difficulty metrics."""
    width_profile = track.width_profile
    curvature = track.centerline.curvature
    
    # Technical score: combination of narrow sections and high curvature
    # Higher score = more technically demanding
    narrow_sections = np.mean(width_profile < 0.6)  # Below 60cm is narrow
    high_curvature_sections = np.mean(np.abs(curvature) > 0.3)
    
    technical_score = (narrow_sections * 0.5 + high_curvature_sections * 0.5) * 100
    
    # Estimate average speed based on curvature
    # Simple model: v_max = sqrt(ay_max / |curvature|) with ay_max = 5 m/s^2 (typical for F1Tenth)
    ay_max = 5.0  # m/s^2
    v_max = np.sqrt(ay_max / (np.abs(curvature) + 1e-6))
    # Clip to reasonable range [1, 10] m/s
    v_max = np.clip(v_max, 1.0, 10.0)
    avg_speed_estimate = np.mean(v_max)
    
    return {
        'technical_score': technical_score,
        'avg_speed_estimate': avg_speed_estimate,
    }


def _compute_racing_metrics(centerline: Any, track: Any, air_ducts: Any) -> Dict[str, Any]:
    """Compute racing-specific metrics like overtaking opportunities."""
    car_width = 0.2  # F1Tenth car width in meters
    safety_buffer = 0.1  # Safety margin
    
    # Overtaking opportunities: where width > 2 * car_width + buffer
    min_overtaking_width = 2 * car_width + safety_buffer
    overtaking_mask = track.width_profile > min_overtaking_width
    
    # Additionally require low curvature for overtaking
    low_curvature_mask = np.abs(centerline.curvature) < 0.1
    overtaking_mask = overtaking_mask & low_curvature_mask
    
    # Count continuous overtaking segments
    overtaking_segments = _find_continuous_segments(overtaking_mask)
    overtaking_opportunities = len(overtaking_segments)
    
    # Ratio of track length where overtaking is possible
    overtaking_length_ratio = np.sum(overtaking_mask) / len(overtaking_mask)
    
    # Recovery slack: distance between racing line and air ducts
    # Simplified: use track width margin
    recovery_slack = np.mean(track.width_profile - min_overtaking_width)
    recovery_slack = max(0, recovery_slack)
    
    return {
        'overtaking_opportunities': overtaking_opportunities,
        'overtaking_length_ratio': overtaking_length_ratio,
        'recovery_slack': recovery_slack,
    }


def _compute_safety_metrics(centerline: Any, track: Any, air_ducts: Any) -> Dict[str, float]:
    """Compute safety-related metrics."""
    # LIDAR feature density: gradient of distance to nearest wall
    # Sample points along centerline
    n_samples = len(centerline.points)
    
    # Distance to left and right walls
    dist_to_left = np.linalg.norm(track.left_boundary - centerline.points, axis=1)
    dist_to_right = np.linalg.norm(track.right_boundary - centerline.points, axis=1)
    dist_to_wall = np.minimum(dist_to_left, dist_to_right)
    
    # Compute gradient of distance to wall (how quickly it changes)
    dist_gradient = np.gradient(dist_to_wall)
    lidar_feature_density = np.mean(np.abs(dist_gradient))
    
    # Add air duct proximity if available
    if air_ducts and air_ducts.obbs:
        # For each centerline point, find distance to nearest duct
        duct_distances = []
        for point in centerline.points:
            min_dist = float('inf')
            for obb in air_ducts.obbs:
                if obb.contains_point(point):
                    min_dist = 0
                    break
                # Distance to OBB corners
                corner_dists = np.linalg.norm(obb.corners - point, axis=1)
                min_dist = min(min_dist, np.min(corner_dists))
            duct_distances.append(min_dist)
        
        # Penalize points too close to ducts
        proximity_penalty = np.mean([max(0, 0.1 - d) for d in duct_distances])
        lidar_feature_density += proximity_penalty
    
    return {
        'lidar_feature_density': lidar_feature_density,
    }


def _compute_self_avoidance_metrics(centerline: Any, track: Any) -> Dict[str, float]:
    """Compute self-avoidance and self-intersection metrics."""
    points = centerline.points
    n = len(points)
    min_distance = float('inf')
    penalty = 0.0
    
    # Check for self-intersections
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
            if _segments_intersect(p1, p2, p3, p4):
                # Find intersection point
                intersection = _line_intersection(p1, p2, p3, p4)
                if intersection is not None:
                    # Compute distance along track
                    dist_to_boundary = min(
                        _point_segment_distance(intersection, points[max(0, i-1)], p1),
                        _point_segment_distance(intersection, p2, points[min(n-1, i+2)])
                    )
                    min_distance = min(min_distance, dist_to_boundary)
                    penalty += max(0, 0.5 - dist_to_boundary)
    
    # Also check track boundaries self-intersection
    boundary_penalty = _check_boundary_self_intersection(track)
    penalty += boundary_penalty
    
    return {
        'self_intersection_penalty': penalty,
        'min_distance_to_self': min_distance if min_distance != float('inf') else 999.0,
    }


def _find_continuous_segments(mask: np.ndarray) -> List[Tuple[int, int]]:
    """Find continuous True segments in a boolean mask."""
    segments = []
    in_segment = False
    start = 0
    
    for i, val in enumerate(mask):
        if val and not in_segment:
            in_segment = True
            start = i
        elif not val and in_segment:
            in_segment = False
            segments.append((start, i - 1))
    
    if in_segment:
        segments.append((start, len(mask) - 1))
    
    return segments


def _segments_intersect(p1: np.ndarray, p2: np.ndarray, 
                        p3: np.ndarray, p4: np.ndarray) -> bool:
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
    if o1 == 0 and _on_segment(p1, p3, p2):
        return True
    if o2 == 0 and _on_segment(p1, p4, p2):
        return True
    if o3 == 0 and _on_segment(p3, p1, p4):
        return True
    if o4 == 0 and _on_segment(p3, p2, p4):
        return True
    
    return False


def _on_segment(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> bool:
    """Check if point q lies on segment pr."""
    return (min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and
            min(p[1], r[1]) <= q[1] <= max(p[1], r[1]))


def _line_intersection(p1: np.ndarray, p2: np.ndarray, 
                       p3: np.ndarray, p4: np.ndarray) -> Optional[np.ndarray]:
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


def _point_segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Compute distance from point p to segment ab."""
    ab = b - a
    ap = p - a
    t = np.dot(ap, ab) / (np.dot(ab, ab) + 1e-6)
    t = np.clip(t, 0, 1)
    projection = a + t * ab
    return np.linalg.norm(p - projection)


def _check_boundary_self_intersection(track: Any) -> float:
    """Check if track boundaries self-intersect."""
    penalty = 0.0
    
    # Check left boundary self-intersection
    left = track.left_boundary
    penalty += _check_curve_self_intersection(left)
    
    # Check right boundary self-intersection
    right = track.right_boundary
    penalty += _check_curve_self_intersection(right)
    
    # Check left-right boundary crossing
    for i in range(len(left) - 1):
        for j in range(len(right) - 1):
            if _segments_intersect(left[i], left[i+1], right[j], right[j+1]):
                penalty += 0.1
    
    return penalty


def _check_curve_self_intersection(points: np.ndarray) -> float:
    """Check if a curve self-intersects."""
    penalty = 0.0
    n = len(points)
    
    for i in range(n - 2):
        for j in range(i + 2, n - 1):
            if j == i + 1:
                continue
            if _segments_intersect(points[i], points[i+1], points[j], points[j+1]):
                penalty += 1.0
    
    return penalty


def compute_metric_summary(metrics_list: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Compute summary statistics for multiple track generations.
    
    Args:
        metrics_list: List of metric dictionaries from multiple runs
        
    Returns:
        Dictionary with mean, std, min, max for each metric
    """
    if not metrics_list:
        return {}
    
    summary = {}
    
    # Get all metric keys (excluding metadata)
    first_metrics = {k: v for k, v in metrics_list[0].items() if k != 'metadata'}
    metric_keys = list(first_metrics.keys())
    
    for key in metric_keys:
        values = [m[key] for m in metrics_list if key in m]
        if values:
            summary[key] = {
                'mean': np.mean(values),
                'std': np.std(values),
                'min': np.min(values),
                'max': np.max(values),
                'count': len(values)
            }
    
    return summary


def compare_metrics(metrics_a: Dict[str, Any], 
                    metrics_b: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
    """
    Compare two sets of metrics.
    
    Args:
        metrics_a: First metric dictionary
        metrics_b: Second metric dictionary
        
    Returns:
        Dictionary with comparison metrics (ratio, difference)
    """
    comparison = {}
    
    # Get all common numeric keys
    keys_a = {k: v for k, v in metrics_a.items() if k != 'metadata' and isinstance(v, (int, float))}
    keys_b = {k: v for k, v in metrics_b.items() if k != 'metadata' and isinstance(v, (int, float))}
    
    common_keys = set(keys_a.keys()) & set(keys_b.keys())
    
    for key in common_keys:
        a_val = keys_a[key]
        b_val = keys_b[key]
        
        if b_val != 0:
            ratio = a_val / b_val
        else:
            ratio = float('inf') if a_val > 0 else 1.0
            
        comparison[key] = {
            'ratio': ratio,
            'difference': a_val - b_val,
            'percent_change': ((a_val - b_val) / (b_val + 1e-6)) * 100
        }
    
    return comparison