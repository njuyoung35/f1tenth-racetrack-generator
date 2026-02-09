import math

import numpy as np

# def track_width_profile(
#     t: float,
#     curvature: float,
#     position: np.ndarray,
#     tangent: np.ndarray,
#     **kwargs
# ) -> float:
#     pass

# **kwargs possible keys:
#     - base_width
#     - total_distance
#     - distance_from_start
#     - distance_ratio
#     - angle


def constant_width_profile(t, curvature, position, tangent, **kwargs):
    return 0.5


def sinusoidal_width_profile(t, curvature, position, tangent, **kwargs):
    """사인파 형태로 변화하는 폭"""
    base_width = 0.5
    variation = 0.2
    frequency = 3  # 트랙을 따라 3번 진동
    return base_width + variation * math.sin(2 * math.pi * frequency * t)


def chicane_profile(t, curvature, position, tangent, **kwargs):
    """특정 구간에서만 폭이 좁아지는 프로파일 (시케인 효과)"""
    base_width = 0.5
    narrow_width = 0.3

    # t = 0.3~0.4, 0.6~0.7 구간에서 좁아짐
    if 0.3 <= t <= 0.4 or 0.6 <= t <= 0.7:
        return narrow_width
    return base_width
