"""
Duct 기반 벽 생성기 - F1Tenth 실제 특성 반영
"""

import os
from typing import Callable, Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.interpolate import splev, splprep
from scipy.spatial import KDTree

# ============================================================================
# 데이터 구조
# ============================================================================


class CenterlinePoint:
    """센터라인 점 데이터 클래스"""

    def __init__(
        self, x, y, t, cumulative_distance, curvature, left_width, right_width
    ):
        self.x = x
        self.y = y
        self.t = t
        self.cumulative_distance = cumulative_distance
        self.curvature = curvature
        self.left_width = left_width  # 왼쪽 최대 허용 길이 (raycast 결과)
        self.right_width = right_width  # 오른쪽 최대 허용 길이 (raycast 결과)

    def to_array(self):
        return np.array([self.x, self.y])


class DuctSegment:
    """Duct 세그먼트 - 하나의 duct가 양쪽 벽을 담당할 수 있음"""

    def __init__(self, segment_id, centerline_points, thickness=0.3):
        self.id = segment_id
        self.centerline = centerline_points  # (N, 2) array
        self.thickness = thickness
        self.wall_points = []  # 이 duct에 할당된 모든 벽 점들

    @property
    def length(self):
        """Duct 길이 계산"""
        if len(self.centerline) < 2:
            return 0.0
        return np.sum(np.linalg.norm(np.diff(self.centerline, axis=0), axis=1))

    @property
    def bounding_box(self):
        """바운딩 박스 반환"""
        points = np.array(self.centerline)
        return {
            "min_x": points[:, 0].min(),
            "max_x": points[:, 0].max(),
            "min_y": points[:, 1].min(),
            "max_y": points[:, 1].max(),
        }


# ============================================================================
# 주요 클래스
# ============================================================================


class WallGenerator:
    """
    F1Tenth 실제 특성을 반영한 벽 생성기

    특징:
    1. 실제 duct 물리적 제약 반영 (두께 0.3m, 최소 트랙 너비 3m)
    2. 하나의 duct가 양쪽 벽을 동시에 담당 가능
    3. Lateral offset function을 통한 유연한 벽 점 생성
    4. Centerline 메타데이터 활용
    """

    def __init__(self, config: Dict):
        """
        Args:
            config: 설정 딕셔너리 (centerline과 동일 config 사용)
        """
        # 기본 파라미터
        self.width = config.get("width", 20.0)
        self.height = config.get("height", 10.0)

        # Duct 특성 (실제 F1Tenth 기준)
        self.duct_thickness = config.get("duct_thickness", 0.30)  # 30cm 두께
        self.min_duct_length = config.get("min_duct_length", 0.50)  # 최소 50cm
        self.max_duct_length = config.get("max_duct_length", 2.00)  # 최대 2m
        self.min_track_width = config.get("min_track_width", 3.00)  # 최소 트랙 너비 3m

        # 벽 생성 파라미터
        self.wall_points_per_meter = config.get("wall_points_per_meter", 50)
        self.connection_tolerance = config.get("connection_tolerance", 0.15)
        self.resolution = config.get("resolution", 0.05)

        # 디버깅
        self.visualize = config.get("visualize", False)
        self.debug_output_dir = config.get("debug_output_dir", "debug")

        # 내부 상태
        self.centerline_points = []  # CenterlinePoint 객체 리스트
        self.duct_segments = []  # DuctSegment 객체 리스트
        self.all_wall_points = None  # 생성된 모든 벽 점들
        self.occupancy_grid = None

        # Lateral offset function (기본값: 일정 너비)
        self.lateral_offset_fn = config.get(
            "lateral_offset_fn", self._default_lateral_offset
        )

        # 디버그 디렉토리 생성
        if self.visualize:
            os.makedirs(self.debug_output_dir, exist_ok=True)

    # ============================================================================
    # 공개 인터페이스
    # ============================================================================

    def generate(self, centerline_data: np.ndarray = None) -> Dict:
        """
        벽 생성 메인 메서드

        Args:
            centerline_data: centerline.py에서 생성된 데이터 (N x 7)
                           [x, y, t, cumulative_distance, curvature, left_width, right_width]

        Returns:
            생성 결과 딕셔너리
        """
        print("=" * 60)
        print("F1Tenth DUCT-BASED WALL GENERATION")
        print(f"  Space: {self.width:.1f}m x {self.height:.1f}m")
        print(f"  Duct thickness: {self.duct_thickness:.2f}m")
        print(f"  Min track width: {self.min_track_width:.1f}m")
        print("=" * 60)

        # 1. 센터라인 데이터 파싱
        if centerline_data is not None:
            self._parse_centerline_data(centerline_data)
        else:
            # 테스트용 데이터 생성
            self._create_test_centerline()

        # 2. Duct 배치 계획 수립
        duct_layout = self._plan_duct_layout()

        # 3. Duct 세그먼트 생성
        self._create_duct_segments(duct_layout)

        # 4. 벽 점 생성 (lateral offset function 사용)
        self._generate_wall_points()

        # 5. 벽 점을 duct에 할당
        self._assign_wall_points_to_ducts()

        # 6. 점유격자 생성
        self.occupancy_grid = self._create_occupancy_grid()

        # 7. 결과 검증
        self._validate_result()

        # 8. 시각화 (옵션)
        if self.visualize:
            self._visualize_generation()

        return self._create_output()

    # ============================================================================
    # 내부 메서드
    # ============================================================================

    def _parse_centerline_data(self, centerline_data: np.ndarray):
        """센터라인 데이터 파싱"""
        print("\n[1/7] Parsing centerline data...")

        self.centerline_points = []
        for i, row in enumerate(centerline_data):
            point = CenterlinePoint(
                x=row[0],
                y=row[1],
                t=row[2],
                cumulative_distance=row[3],
                curvature=row[4],
                left_width=row[5],
                right_width=row[6],
            )
            self.centerline_points.append(point)

        print(f"  - Loaded {len(self.centerline_points)} centerline points")
        print(
            f"  - Track length: {self.centerline_points[-1].cumulative_distance:.1f}m"
        )

        # 왼쪽/오른쪽 최대 너비 통계
        avg_left = np.mean([p.left_width for p in self.centerline_points])
        avg_right = np.mean([p.right_width for p in self.centerline_points])
        print(f"  - Avg left width: {avg_left:.1f}m, Avg right width: {avg_right:.1f}m")

    def _create_test_centerline(self):
        """테스트용 센터라인 생성"""
        print("\n[1/7] Creating test centerline...")

        # 타원형 트랙 생성
        n_points = 100
        theta = np.linspace(0, 2 * np.pi, n_points)

        # 공간 내에 맞도록 크기 조정
        cx, cy = self.width / 2, self.height / 2
        rx, ry = self.width / 2 * 0.7, self.height / 2 * 0.7

        self.centerline_points = []
        cumulative_distance = 0.0

        for i, t in enumerate(theta):
            x = cx + rx * np.cos(t)
            y = cy + ry * np.sin(t)

            if i > 0:
                prev = self.centerline_points[-1]
                dx = x - prev.x
                dy = y - prev.y
                cumulative_distance += np.sqrt(dx * dx + dy * dy)

            point = CenterlinePoint(
                x=x,
                y=y,
                t=i / (n_points - 1),
                cumulative_distance=cumulative_distance,
                curvature=0.1 + 0.05 * np.sin(t * 3),  # 가변 곡률
                left_width=1.8,  # 왼쪽 최대 너비
                right_width=1.5,  # 오른쪽 최대 너비
            )
            self.centerline_points.append(point)

        print(f"  - Created {len(self.centerline_points)} test centerline points")

    def _plan_duct_layout(self) -> List[Dict]:
        """Duct 배치 계획 수립"""
        print("\n[2/7] Planning duct layout...")

        total_length = self.centerline_points[-1].cumulative_distance
        n_ducts = int(np.ceil(total_length / self.max_duct_length))
        duct_length = total_length / n_ducts

        # Duct 길이 제한 적용
        duct_length = np.clip(duct_length, self.min_duct_length, self.max_duct_length)
        n_ducts = int(np.ceil(total_length / duct_length))

        duct_layout = []
        current_distance = 0.0

        for i in range(n_ducts):
            start_dist = current_distance
            end_dist = min(current_distance + duct_length, total_length)

            # 센터라인 세그먼트 추출
            seg_points = []
            for p in self.centerline_points:
                if start_dist <= p.cumulative_distance <= end_dist:
                    seg_points.append([p.x, p.y])

            if len(seg_points) < 2:
                continue

            seg_array = np.array(seg_points)

            duct_layout.append(
                {
                    "id": i,
                    "start_distance": start_dist,
                    "end_distance": end_dist,
                    "centerline": seg_array,
                    "length": np.sum(
                        np.linalg.norm(np.diff(seg_array, axis=0), axis=1)
                    ),
                }
            )

            current_distance = end_dist

        print(f"  - Planned {len(duct_layout)} duct segments")
        print(
            f"  - Average duct length: {np.mean([d['length'] for d in duct_layout]):.2f}m"
        )

        return duct_layout

    def _create_duct_segments(self, duct_layout: List[Dict]):
        """Duct 세그먼트 생성"""
        print("\n[3/7] Creating duct segments...")

        self.duct_segments = []
        for layout in duct_layout:
            duct = DuctSegment(
                segment_id=layout["id"],
                centerline_points=layout["centerline"],
                thickness=self.duct_thickness,
            )
            self.duct_segments.append(duct)

        print(f"  - Created {len(self.duct_segments)} duct segments")

    def _default_lateral_offset(self, **kwargs) -> float:
        """
        기본 lateral offset function

        Args:
            **kwargs: 가능한 파라미터들
                - x, y: 현재 점 좌표
                - t: 파라미터 [0,1]
                - cumulative_distance: 누적 거리
                - curvature: 곡률
                - left_width: 왼쪽 최대 허용 길이
                - right_width: 오른쪽 최대 허용 길이
                - base_width: 기본 트랙 너비
                - is_left: 왼쪽 벽인지 여부

        Returns:
            offset: 중심선으로부터의 거리
        """
        # 기본값: 왼쪽/오른쪽 최대 너비의 80% 위치에 벽 배치
        max_width = (
            kwargs.get("left_width", 1.5)
            if kwargs.get("is_left", True)
            else kwargs.get("right_width", 1.5)
        )
        return max_width * 0.8

    def _generate_wall_points(self):
        """벽 점 생성"""
        print("\n[4/7] Generating wall points...")

        all_wall_points = []

        for i, cl_point in enumerate(self.centerline_points):
            # 접선 벡터 계산 (이전/다음 점 사용)
            if i == 0:
                next_point = self.centerline_points[i + 1]
                tangent = np.array(
                    [next_point.x - cl_point.x, next_point.y - cl_point.y]
                )
            elif i == len(self.centerline_points) - 1:
                prev_point = self.centerline_points[i - 1]
                tangent = np.array(
                    [cl_point.x - prev_point.x, cl_point.y - prev_point.y]
                )
            else:
                next_point = self.centerline_points[i + 1]
                prev_point = self.centerline_points[i - 1]
                tangent = np.array(
                    [next_point.x - prev_point.x, next_point.y - prev_point.y]
                )

            # 정규화
            tangent_norm = np.linalg.norm(tangent)
            if tangent_norm > 0:
                tangent = tangent / tangent_norm

            # 수직 벡터 (왼쪽 기준)
            normal = np.array([-tangent[1], tangent[0]])

            # 왼쪽 벽 점 생성
            left_offset = self.lateral_offset_fn(
                x=cl_point.x,
                y=cl_point.y,
                t=cl_point.t,
                cumulative_distance=cl_point.cumulative_distance,
                curvature=cl_point.curvature,
                left_width=cl_point.left_width,
                right_width=cl_point.right_width,
                base_width=self.min_track_width / 2,
                is_left=True,
            )

            # 왼쪽 최대 너비 내에서만 생성
            left_offset = min(left_offset, cl_point.left_width * 0.95)
            left_point = np.array([cl_point.x, cl_point.y]) + normal * left_offset

            # 오른쪽 벽 점 생성
            right_offset = self.lateral_offset_fn(
                x=cl_point.x,
                y=cl_point.y,
                t=cl_point.t,
                cumulative_distance=cl_point.cumulative_distance,
                curvature=cl_point.curvature,
                left_width=cl_point.left_width,
                right_width=cl_point.right_width,
                base_width=self.min_track_width / 2,
                is_left=False,
            )

            # 오른쪽 최대 너비 내에서만 생성
            right_offset = min(right_offset, cl_point.right_width * 0.95)
            right_point = np.array([cl_point.x, cl_point.y]) - normal * right_offset

            # 점들 추가 (약간의 노이즈로 밀도 증가)
            n_points_per_side = max(
                2, int(self.wall_points_per_meter * min(left_offset, right_offset) / 10)
            )

            for j in range(n_points_per_side):
                # 왼쪽 점 주변에 노이즈 추가
                if j > 0:
                    noise = np.random.normal(0, self.duct_thickness / 10, 2)
                    all_wall_points.append(left_point + noise)

                # 오른쪽 점 주변에 노이즈 추가
                noise = np.random.normal(0, self.duct_thickness / 10, 2)
                all_wall_points.append(right_point + noise)

        self.all_wall_points = np.array(all_wall_points)
        print(f"  - Generated {len(self.all_wall_points)} wall points")

    def _assign_wall_points_to_ducts(self):
        """벽 점들을 가장 가까운 duct에 할당"""
        print("\n[5/7] Assigning wall points to ducts...")

        if len(self.all_wall_points) == 0:
            print("  ⚠️  No wall points to assign")
            return

        # KD-tree로 효율적인 할당
        wall_kdtree = KDTree(self.all_wall_points)

        for duct in self.duct_segments:
            bbox = duct.bounding_box
            margin = self.duct_thickness * 3.0

            # duct 중앙점
            center_idx = len(duct.centerline) // 2
            duct_center = duct.centerline[center_idx]

            # duct 중심으로부터 특정 반경 내 점들 찾기
            search_radius = duct.length / 2 + margin
            indices = wall_kdtree.query_ball_point(duct_center, search_radius)

            if indices:
                duct_points = self.all_wall_points[indices]

                # 각 점에서 duct 중심선까지의 거리 계산
                distances = []
                for point in duct_points:
                    min_dist = float("inf")
                    for i in range(len(duct.centerline) - 1):
                        dist = self._point_to_segment_distance(
                            point, duct.centerline[i], duct.centerline[i + 1]
                        )
                        min_dist = min(min_dist, dist)
                    distances.append(min_dist)

                distances = np.array(distances)

                # duct 두께의 1.5배 내의 점들만 할당
                mask = distances <= (self.duct_thickness * 1.5)
                duct.wall_points = duct_points[mask].tolist()

                print(
                    f"  - Duct {duct.id}: assigned {len(duct.wall_points)} points "
                    f"(avg distance: {np.mean(distances[mask]):.2f}m)"
                )

    def _point_to_segment_distance(self, point, seg_start, seg_end):
        """점과 선분 사이의 거리 계산"""
        line_vec = seg_end - seg_start
        point_vec = point - seg_start
        line_len = np.dot(line_vec, line_vec)

        if line_len == 0:
            return np.linalg.norm(point - seg_start)

        t = max(0, min(1, np.dot(point_vec, line_vec) / line_len))
        projection = seg_start + t * line_vec
        return np.linalg.norm(point - projection)

    # def _create_occupancy_grid(self) -> np.ndarray:
    #     """점유격자 생성"""
    #     print("\n[6/7] Creating occupancy grid...")

    #     if len(self.all_wall_points) == 0:
    #         print("  ⚠️  No wall points to create occupancy grid")
    #         return np.array([])

    #     # 경계 계산 (전체 공간 사용)
    #     x_min, x_max = 0, self.width
    #     y_min, y_max = 0, self.height

    #     # 그리드 크기 계산
    #     grid_cols = int(np.ceil((x_max - x_min) / self.resolution))
    #     grid_rows = int(np.ceil((y_max - y_min) / self.resolution))

    #     occupancy = np.zeros((grid_rows, grid_cols), dtype=np.uint8)

    #     print(
    #         f"  - Grid size: {grid_cols}x{grid_rows} "
    #         f"({grid_cols * self.resolution:.1f}m x {grid_rows * self.resolution:.1f}m)"
    #     )

    #     # 모든 벽 점들을 그리드에 표시
    #     for point in self.all_wall_points:
    #         grid_x = int((point[0] - x_min) / self.resolution)
    #         grid_y = int((point[1] - y_min) / self.resolution)

    #         if 0 <= grid_x < grid_cols and 0 <= grid_y < grid_rows:
    #             # duct 두께 고려 (반지름 픽셀 수)
    #             radius_pixels = int(
    #                 np.ceil(self.duct_thickness / (2 * self.resolution))
    #             )

    #             # 원형으로 채우기
    #             for dx in range(-radius_pixels, radius_pixels + 1):
    #                 for dy in range(-radius_pixels, radius_pixels + 1):
    #                     px = grid_x + dx
    #                     py = grid_y + dy

    #                     if 0 <= px < grid_cols and 0 <= py < grid_rows:
    #                         # 원형 마스크
    #                         if dx * dx + dy * dy <= radius_pixels * radius_pixels:
    #                             occupancy[py, px] = 1

    #     # 센터라인 영역은 비움 (주행 가능 영역)
    #     for cl_point in self.centerline_points:
    #         grid_x = int((cl_point.x - x_min) / self.resolution)
    #         grid_y = int((cl_point.y - y_min) / self.resolution)

    #         if 0 <= grid_x < grid_cols and 0 <= grid_y < grid_rows:
    #             # 트랙 너비의 절반만큼 제거
    #             track_radius = int(self.min_track_width / (2 * self.resolution))

    #             for dx in range(-track_radius, track_radius + 1):
    #                 for dy in range(-track_radius, track_radius + 1):
    #                     px = grid_x + dx
    #                     py = grid_y + dy

    #                     if 0 <= px < grid_cols and 0 <= py < grid_rows:
    #                         if dx * dx + dy * dy <= track_radius * track_radius:
    #                             occupancy[py, px] = 0

    #     wall_cells = np.sum(occupancy)
    #     total_cells = grid_cols * grid_rows
    #     print(
    #         f"  - Wall cells: {wall_cells}/{total_cells} "
    #         f"({wall_cells / total_cells * 100:.1f}%)"
    #     )

    #     return occupancy

    def _create_occupancy_grid(self) -> np.ndarray:
        """점유격자 생성 (수정 버전)"""
        print("\n[6/7] Creating occupancy grid...")

        if len(self.all_wall_points) == 0:
            print("  ⚠️  No wall points to create occupancy grid")
            return np.array([])

        # 1. 실제 벽 점들의 범위 계산 (전체 공간이 아닌!)
        if len(self.all_wall_points) > 0:
            x_min = self.all_wall_points[:, 0].min()
            x_max = self.all_wall_points[:, 0].max()
            y_min = self.all_wall_points[:, 1].min()
            y_max = self.all_wall_points[:, 1].max()
        else:
            # 백업: 전체 공간 사용
            x_min, x_max = 0, self.width
            y_min, y_max = 0, self.height

        # 여유 공간 추가 (duct 두께의 2배)
        margin = self.duct_thickness * 2.0
        x_min -= margin
        x_max += margin
        y_min -= margin
        y_max += margin

        # 경계를 전체 공간 내로 제한
        x_min = max(0, x_min)
        x_max = min(self.width, x_max)
        y_min = max(0, y_min)
        y_max = min(self.height, y_max)

        # 2. 그리드 크기 계산
        grid_cols = int(np.ceil((x_max - x_min) / self.resolution))
        grid_rows = int(np.ceil((y_max - y_min) / self.resolution))

        print(
            f"  - Wall points bounding box: "
            f"[{x_min:.1f}, {y_min:.1f}] to [{x_max:.1f}, {y_max:.1f}]"
        )
        print(
            f"  - Grid size: {grid_cols}x{grid_rows} "
            f"({(x_max - x_min):.1f}m x {(y_max - y_min):.1f}m)"
        )

        # 3. 점유격자 초기화
        occupancy = np.zeros((grid_rows, grid_cols), dtype=np.uint8)

        # 4. 모든 벽 점들을 그리드에 표시 (더 효율적인 방법)
        if len(self.all_wall_points) > 0:
            # 벽 점들을 그리드 좌표로 변환
            grid_xs = ((self.all_wall_points[:, 0] - x_min) / self.resolution).astype(
                int
            )
            grid_ys = ((self.all_wall_points[:, 1] - y_min) / self.resolution).astype(
                int
            )

            # 유효한 좌표만 필터링
            valid_mask = (
                (grid_xs >= 0)
                & (grid_xs < grid_cols)
                & (grid_ys >= 0)
                & (grid_ys < grid_rows)
            )

            grid_xs = grid_xs[valid_mask]
            grid_ys = grid_ys[valid_mask]

            print(
                f"  - Valid points for grid: {len(grid_xs)}/{len(self.all_wall_points)}"
            )

            if len(grid_xs) > 0:
                # 각 점을 중심으로 duct 두께만큼 채우기
                radius_pixels = int(np.ceil(self.duct_thickness / self.resolution))

                # 효율적인 마스크 생성
                for i in range(len(grid_xs)):
                    x, y = grid_xs[i], grid_ys[i]

                    # 원형 마스크 범위 계산
                    x_start = max(0, x - radius_pixels)
                    x_end = min(grid_cols, x + radius_pixels + 1)
                    y_start = max(0, y - radius_pixels)
                    y_end = min(grid_rows, y + radius_pixels + 1)

                    # 원형 마스크 적용
                    for gy in range(y_start, y_end):
                        for gx in range(x_start, x_end):
                            dx = gx - x
                            dy = gy - y
                            if dx * dx + dy * dy <= radius_pixels * radius_pixels:
                                occupancy[gy, gx] = 1

        # 5. Duct에 할당된 점들도 표시 (디버깅용)
        for duct in self.duct_segments:
            if len(duct.wall_points) > 0:
                duct_points = np.array(duct.wall_points)
                if len(duct_points) > 0:
                    duct_grid_xs = (
                        (duct_points[:, 0] - x_min) / self.resolution
                    ).astype(int)
                    duct_grid_ys = (
                        (duct_points[:, 1] - y_min) / self.resolution
                    ).astype(int)

                    valid_mask = (
                        (duct_grid_xs >= 0)
                        & (duct_grid_xs < grid_cols)
                        & (duct_grid_ys >= 0)
                        & (duct_grid_ys < grid_rows)
                    )

                    duct_grid_xs = duct_grid_xs[valid_mask]
                    duct_grid_ys = duct_grid_ys[valid_mask]

                    # Duct 점들은 다른 값으로 표시 (디버깅용)
                    for x, y in zip(duct_grid_xs, duct_grid_ys):
                        if 0 <= x < grid_cols and 0 <= y < grid_rows:
                            occupancy[y, x] = 2  # duct 점들은 값 2

        # 6. 통계 계산
        wall_cells = np.sum(occupancy == 1)
        duct_cells = np.sum(occupancy == 2)
        total_cells = grid_rows * grid_cols

        print(
            f"  - Wall cells: {wall_cells}/{total_cells} "
            f"({wall_cells / total_cells * 100:.1f}%)"
        )
        print(f"  - Duct cells: {duct_cells} (assigned points)")
        print(f"  - Total occupied: {wall_cells + duct_cells}")

        return occupancy

    def _validate_result(self):
        """결과 검증"""
        print("\n[7/7] Validating result...")

        # 1. 충분한 벽 점 생성 확인
        if len(self.all_wall_points) < 100:
            print(
                f"  ⚠️  Warning: Only {len(self.all_wall_points)} wall points generated"
            )

        # 2. Duct 할당 확인
        total_assigned = sum(len(d.wall_points) for d in self.duct_segments)
        if total_assigned < len(self.all_wall_points) * 0.8:
            print(
                f"  ⚠️  Warning: Only {total_assigned}/{len(self.all_wall_points)} "
                f"points assigned to ducts"
            )

        # 3. 트랙 너비 검증
        if len(self.centerline_points) > 0:
            min_width = float("inf")
            for p in self.centerline_points:
                total_width = p.left_width + p.right_width
                min_width = min(min_width, total_width)

            print(f"  - Minimum available width: {min_width:.1f}m")
            if min_width < self.min_track_width:
                print(
                    f"  ⚠️  Warning: Minimum width ({min_width:.1f}m) "
                    f"< required ({self.min_track_width:.1f}m)"
                )
            else:
                print(
                    f"  ✓ Track width OK: {min_width:.1f}m >= {self.min_track_width:.1f}m"
                )

    def _create_output(self) -> Dict:
        """출력 데이터 생성"""
        output = {
            "wall_points": self.all_wall_points,
            "occupancy_grid": self.occupancy_grid,
            "duct_segments": self.duct_segments,
            "metadata": {
                "generator": "DuctBasedWallGenerator",
                "space_size": (self.width, self.height),
                "duct_thickness": self.duct_thickness,
                "min_track_width": self.min_track_width,
                "num_wall_points": len(self.all_wall_points),
                "num_ducts": len(self.duct_segments),
                "total_duct_length": sum(d.length for d in self.duct_segments),
            },
        }

        # centerline 데이터 추가 (main.py에서 필요할 수 있음)
        centerline_array = np.array(
            [
                [
                    p.x,
                    p.y,
                    p.t,
                    p.cumulative_distance,
                    p.curvature,
                    p.left_width,
                    p.right_width,
                ]
                for p in self.centerline_points
            ]
        )
        output["centerline"] = centerline_array

        return output

    # ============================================================================
    # 시각화 메서드
    # ============================================================================

    def _visualize_generation(self):
        """생성 과정 시각화"""
        print("\n[Visualization] Creating visualization plots...")

        # 1. 전체 레이아웃
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))

        # 1-1. 센터라인과 duct 배치
        ax1 = axes[0, 0]
        ax1.set_title("1. Centerline and Duct Layout")

        # 공간 경계
        ax1.plot(
            [0, self.width, self.width, 0, 0],
            [0, 0, self.height, self.height, 0],
            "k-",
            linewidth=2,
            label="Space Boundary",
        )

        # 센터라인
        centerline_array = np.array([[p.x, p.y] for p in self.centerline_points])
        ax1.plot(
            centerline_array[:, 0],
            centerline_array[:, 1],
            "b-",
            alpha=0.5,
            linewidth=1,
            label="Centerline",
        )

        # Duct 세그먼트
        colors = plt.cm.Set3(np.linspace(0, 1, len(self.duct_segments)))
        for i, duct in enumerate(self.duct_segments):
            duct_array = np.array(duct.centerline)
            ax1.plot(
                duct_array[:, 0],
                duct_array[:, 1],
                color=colors[i],
                linewidth=2,
                alpha=0.7,
            )
            ax1.scatter(
                duct_array[0, 0], duct_array[0, 1], color=colors[i], s=50, marker=">"
            )
            ax1.text(
                duct_array[0, 0],
                duct_array[0, 1],
                f"D{duct.id}",
                fontsize=8,
                ha="right",
            )

        ax1.legend()
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.axis("equal")
        ax1.grid(True, alpha=0.3)

        # # 1-2. 생성된 벽 점들
        # ax2 = axes[0, 1]
        # ax2.set_title("2. Generated Wall Points")

        # ax2.plot(
        #     centerline_array[:, 0], centerline_array[:, 1], "b-", alpha=0.3, linewidth=1
        # )

        # if self.all_wall_points is not None and len(self.all_wall_points) > 0:
        #     ax2.scatter(
        #         self.all_wall_points[:, 0],
        #         self.all_wall_points[:, 1],
        #         s=1,
        #         c="red",
        #         alpha=0.5,
        #         label="Wall Points",
        #     )

        # ax2.set_xlabel("X (m)")
        # ax2.set_ylabel("Y (m)")
        # ax2.axis("equal")
        # ax2.grid(True, alpha=0.3)
        #
        # 1-2. 생성된 벽 점들 (간단한 버전)
        ax2 = axes[0, 1]
        ax2.set_title("2. Wall Points with Normals")

        # 센터라인 그리기
        ax2.plot(
            centerline_array[:, 0],
            centerline_array[:, 1],
            "b-",
            alpha=0.6,
            linewidth=1.5,
            label="Centerline",
        )

        if self.all_wall_points is not None and len(self.all_wall_points) > 0:
            # 법선 방향에 따라 왼쪽/오른쪽 벽 점 분류
            left_points = []
            right_points = []

            # 일부 센터라인 점에서만 시각화 (너무 많지 않게)
            step = max(1, len(self.centerline_points) // 25)

            for i in range(0, len(self.centerline_points), step):
                cl_point = self.centerline_points[i]
                center = np.array([cl_point.x, cl_point.y])

                # tangent 계산
                if i == 0:
                    next_point = self.centerline_points[i + 1]
                    tangent = np.array(
                        [next_point.x - cl_point.x, next_point.y - cl_point.y]
                    )
                elif i == len(self.centerline_points) - 1:
                    prev_point = self.centerline_points[i - 1]
                    tangent = np.array(
                        [cl_point.x - prev_point.x, cl_point.y - prev_point.y]
                    )
                else:
                    next_point = self.centerline_points[i + 1]
                    prev_point = self.centerline_points[i - 1]
                    tangent = np.array(
                        [next_point.x - prev_point.x, next_point.y - prev_point.y]
                    )

                # 정규화
                tangent_norm = np.linalg.norm(tangent)
                if tangent_norm > 0:
                    tangent = tangent / tangent_norm

                # 법선 벡터
                normal = np.array([-tangent[1], tangent[0]])

                # 가장 가까운 벽 점 찾기
                if len(self.all_wall_points) > 0:
                    distances = np.linalg.norm(self.all_wall_points - center, axis=1)
                    closest_idx = np.argmin(distances)
                    closest_point = self.all_wall_points[closest_idx]

                    # 왼쪽/오른쪽 판별
                    wall_vector = closest_point - center
                    if np.dot(wall_vector, normal) > 0:
                        left_points.append(closest_point)
                        color = "green"
                    else:
                        right_points.append(closest_point)
                        color = "red"

                    # 연결선 그리기
                    ax2.plot(
                        [cl_point.x, closest_point[0]],
                        [cl_point.y, closest_point[1]],
                        color=color,
                        linewidth=0.8,
                        alpha=0.4,
                    )

                    # 법선벡터 그리기
                    scale = distances[closest_idx] * 0.4
                    ax2.arrow(
                        cl_point.x,
                        cl_point.y,
                        normal[0] * scale,
                        normal[1] * scale,
                        head_width=0.1,
                        head_length=0.15,
                        fc="orange",
                        ec="orange",
                        alpha=0.7,
                    )

                    # 센터라인 점 표시
                    ax2.scatter(
                        cl_point.x, cl_point.y, s=30, c="blue", marker="o", zorder=5
                    )

            # 왼쪽 벽 점들
            if left_points:
                left_array = np.array(left_points)
                ax2.scatter(
                    left_array[:, 0],
                    left_array[:, 1],
                    s=15,
                    c="green",
                    alpha=0.6,
                    label="Left Wall",
                )

            # 오른쪽 벽 점들
            if right_points:
                right_array = np.array(right_points)
                ax2.scatter(
                    right_array[:, 0],
                    right_array[:, 1],
                    s=15,
                    c="red",
                    alpha=0.6,
                    label="Right Wall",
                )

        ax2.set_xlabel("X (m)")
        ax2.set_ylabel("Y (m)")
        ax2.axis("equal")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="upper right")

        # 1-3. Duct에 할당된 점들
        ax3 = axes[1, 0]
        ax3.set_title("3. Points Assigned to Ducts")

        ax3.plot(
            centerline_array[:, 0], centerline_array[:, 1], "b-", alpha=0.3, linewidth=1
        )

        for i, duct in enumerate(self.duct_segments):
            if len(duct.wall_points) > 0:
                points_array = np.array(duct.wall_points)
                ax3.scatter(
                    points_array[:, 0],
                    points_array[:, 1],
                    s=2,
                    color=colors[i],
                    alpha=0.6,
                    label=f"Duct {duct.id}",
                )

        ax3.set_xlabel("X (m)")
        ax3.set_ylabel("Y (m)")
        ax3.axis("equal")
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="upper right", fontsize=8)

        # 1-4. 점유격자
        ax4 = axes[1, 1]
        ax4.set_title("4. Occupancy Grid")

        if self.occupancy_grid is not None and self.occupancy_grid.size > 0:
            ax4.imshow(
                self.occupancy_grid,
                cmap="binary_r",
                extent=[0, self.width, 0, self.height],
                origin="lower",
                alpha=0.7,
            )

            # 센터라인 오버레이
            ax4.plot(
                centerline_array[:, 0],
                centerline_array[:, 1],
                "g-",
                linewidth=1,
                alpha=0.7,
                label="Centerline",
            )

        ax4.set_xlabel("X (m)")
        ax4.set_ylabel("Y (m)")
        ax4.axis("equal")
        ax4.grid(True, alpha=0.3)
        ax4.legend()

        plt.tight_layout()
        plt.savefig(
            os.path.join(self.debug_output_dir, "wall_generation.png"),
            dpi=150,
            bbox_inches="tight",
        )
        plt.close()

        # 2. 상세 통계 그래프
        fig2, axes2 = plt.subplots(1, 3, figsize=(15, 4))

        # 2-1. 트랙 너비 분포
        ax1 = axes2[0]
        left_widths = [p.left_width for p in self.centerline_points]
        right_widths = [p.right_width for p in self.centerline_points]

        distances = [p.cumulative_distance for p in self.centerline_points]
        ax1.plot(distances, left_widths, "b-", label="Left Width")
        ax1.plot(distances, right_widths, "r-", label="Right Width")
        ax1.axhline(
            y=self.min_track_width / 2,
            color="g",
            linestyle="--",
            label=f"Min Width/2 ({self.min_track_width / 2:.1f}m)",
        )

        ax1.set_xlabel("Distance along centerline (m)")
        ax1.set_ylabel("Width (m)")
        ax1.set_title("Track Width Distribution")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # 2-2. Duct 길이 분포
        ax2 = axes2[1]
        duct_lengths = [d.length for d in self.duct_segments]
        duct_ids = [d.id for d in self.duct_segments]

        ax2.bar(duct_ids, duct_lengths, color=colors[: len(duct_ids)])
        ax2.axhline(
            y=self.min_duct_length,
            color="r",
            linestyle="--",
            label=f"Min ({self.min_duct_length:.1f}m)",
        )
        ax2.axhline(
            y=self.max_duct_length,
            color="r",
            linestyle="--",
            label=f"Max ({self.max_duct_length:.1f}m)",
        )

        ax2.set_xlabel("Duct ID")
        ax2.set_ylabel("Length (m)")
        ax2.set_title("Duct Length Distribution")
        ax2.legend()
        ax2.grid(True, alpha=0.3, axis="y")

        # 2-3. 점 할당 통계
        ax3 = axes2[2]
        duct_points_counts = [len(d.wall_points) for d in self.duct_segments]

        ax3.bar(duct_ids, duct_points_counts, color=colors[: len(duct_ids)])
        ax3.set_xlabel("Duct ID")
        ax3.set_ylabel("Number of Points")
        ax3.set_title("Points per Duct")
        ax3.grid(True, alpha=0.3, axis="y")

        plt.tight_layout()
        plt.savefig(
            os.path.join(self.debug_output_dir, "wall_statistics.png"),
            dpi=150,
            bbox_inches="tight",
        )
        plt.close()

        print(f"  - Visualizations saved to {self.debug_output_dir}/")


# ============================================================================
# 사용 예시
# ============================================================================


def example_lateral_offset_fn(**kwargs):
    """
    커스텀 lateral offset function 예시

    곡률이 클수록 트랙을 넓히는 함수
    """
    base_offset = kwargs.get("base_width", 1.5)
    curvature = abs(kwargs.get("curvature", 0))
    is_left = kwargs.get("is_left", True)

    # 곡률에 따른 보정: 곡률이 클수록 더 넓은 트랙
    curvature_factor = 1.0 + curvature * 2.0

    # 왼쪽/오른쪽 비대칭
    side_factor = 1.1 if is_left else 0.9

    return base_offset * curvature_factor * side_factor


if __name__ == "__main__":
    # 테스트 실행
    config = {
        "width": 20.0,
        "height": 10.0,
        "duct_thickness": 0.30,
        "min_track_width": 3.00,
        "visualize": True,
        "debug_output_dir": "debug_walls",
        "lateral_offset_fn": example_lateral_offset_fn,
    }

    generator = WallGenerator(config)
    result = generator.generate()

    print("\n" + "=" * 60)
    print("GENERATION COMPLETE!")
    print("=" * 60)
    print(f"Total wall points: {len(result['wall_points'])}")
    print(
        f"Occupancy grid shape: {result['occupancy_grid'].shape if result['occupancy_grid'].size > 0 else 'Empty'}"
    )
    print(f"Number of ducts: {len(result['duct_segments'])}")
