from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splev, splprep
from scipy.spatial import KDTree

# ============================================================================
# 1. 데이터 구조 정의
# ============================================================================


@dataclass
class CenterlinePoint:
    """센터라인 점 데이터 구조"""

    x: float
    y: float
    t: float  # 파라미터 [0,1]
    cumulative_distance: float  # 누적 거리
    curvature: float  # 곡률
    left_width: float  # 왼쪽 최대 너비
    right_width: float  # 오른쪽 최대 너비

    def to_array(self):
        return np.array([self.x, self.y])


@dataclass
class DuctSegment:
    """Duct 세그먼트 데이터 구조"""

    id: int
    start_point: np.ndarray  # 시작점 (x, y)
    end_point: np.ndarray  # 끝점 (x, y)
    centerline: List[np.ndarray]  # duct 중심선
    assigned_points: List[np.ndarray]  # 할당된 벽 점들
    thickness: float = 0.15  # duct 두께 (기본값 15cm)
    max_curvature: float = 0.3  # 최대 곡률 (물리적 제한)
    is_straight: bool = False  # 직선/곡선 구분

    def length(self) -> float:
        """Duct 길이 계산"""
        return np.sum(np.linalg.norm(np.diff(self.centerline, axis=0), axis=1))

    def bounding_box(self) -> Tuple[float, float, float, float]:
        """바운딩 박스 반환 (min_x, max_x, min_y, max_y)"""
        points = np.array(self.centerline)
        return (
            points[:, 0].min(),
            points[:, 0].max(),
            points[:, 1].min(),
            points[:, 1].max(),
        )


@dataclass
class DuctConnection:
    """Duct 연결점 데이터 구조"""

    from_duct: int  # 출발 duct ID
    to_duct: int  # 도착 duct ID
    connection_point: np.ndarray  # 연결점
    angle_diff: float  # 연결 각도 차이


# ============================================================================
# 2. 주요 클래스 설계
# ============================================================================


class DuctBasedWallGenerator:
    """
    F1Tenth의 물리적 특성을 반영한 Duct 기반 벽 생성기

    주요 특징:
    1. Duct의 물리적 제약 반영 (최소 곡률 반경, 최대 길이 등)
    2. 실제 설치 방식을 모방한 공간 분할
    3. 하나의 duct가 여러 벽 영역을 담당할 수 있음
    4. Centerline 메타데이터 최대한 활용
    """

    def __init__(self, config: Dict):
        """
        Args:
            config: 생성기 설정
                - duct_thickness: duct 두께 (m, 기본 0.15)
                - min_duct_length: 최소 duct 길이 (m, 기본 0.3)
                - max_duct_length: 최대 duct 길이 (m, 기본 1.0)
                - max_curvature: 최대 곡률 (1/m, 기본 0.3)
                - min_track_width: 최소 트랙 너비 (m, 기본 0.8)
                - connection_tolerance: 연결 허용 오차 (m, 기본 0.1)
                - resolution: 점유격자 해상도 (m, 기본 0.05)
        """
        self.duct_thickness = config.get("duct_thickness", 0.15)
        self.min_duct_length = config.get("min_duct_length", 0.3)
        self.max_duct_length = config.get("max_duct_length", 1.0)
        self.max_curvature = config.get("max_curvature", 0.3)
        self.min_track_width = config.get("min_track_width", 0.8)
        self.connection_tolerance = config.get("connection_tolerance", 0.1)
        self.resolution = config.get("resolution", 0.05)

        # 내부 상태
        self.centerline: List[CenterlinePoint] = []
        self.wall_points: np.ndarray = None
        self.duct_segments: List[DuctSegment] = []
        self.duct_connections: List[DuctConnection] = []

    # ============================================================================
    # 3. 주요 공개 메서드
    # ============================================================================

    def generate_walls(
        self,
        centerline: np.ndarray,
        wall_points: np.ndarray,
        centerline_metadata: Optional[Dict] = None,
    ) -> Dict:
        """
        벽 생성 메인 메서드

        Args:
            centerline: (N, 2) 형태의 센터라인 점들
            wall_points: (M, 2) 형태의 벽 점들
            centerline_metadata: 센터라인 메타데이터 (선택적)

        Returns:
            생성 결과 딕셔너리
        """
        print("=" * 60)
        print("DUCT-BASED WALL GENERATION STARTED")
        print("=" * 60)

        # 1. 데이터 전처리
        self._preprocess_inputs(centerline, wall_points, centerline_metadata)

        # 2. Duct 배치 계획 수립
        duct_plan = self._plan_duct_layout()

        # 3. Duct 세그먼트 생성
        self._create_duct_segments(duct_plan)

        # 4. 벽 점들을 duct에 할당
        self._assign_points_to_ducts()

        # 5. Duct 연결 최적화
        self._optimize_duct_connections()

        # 6. 최종 벽 생성
        walls = self._generate_final_walls()

        # 7. 결과 검증
        self._validate_walls(walls)

        return {
            "duct_segments": self.duct_segments,
            "duct_connections": self.duct_connections,
            "walls": walls,
            "occupancy_grid": self._create_occupancy_grid(walls),
            "metadata": self._generate_metadata(),
        }

    # ============================================================================
    # 4. 내부 처리 메서드 (단계별 구현)
    # ============================================================================

    def _preprocess_inputs(
        self, centerline: np.ndarray, wall_points: np.ndarray, metadata: Optional[Dict]
    ):
        """입력 데이터 전처리"""
        print("Step 1: Preprocessing inputs...")

        # Centerline을 CenterlinePoint 리스트로 변환
        self.centerline = []
        n_points = len(centerline)

        for i, (x, y) in enumerate(centerline):
            point = CenterlinePoint(
                x=x,
                y=y,
                t=i / (n_points - 1) if n_points > 1 else 0,
                cumulative_distance=self._calculate_cumulative_distance(centerline, i),
                curvature=metadata["curvature"][i]
                if metadata and "curvature" in metadata
                else 0,
                left_width=metadata["left_width"][i]
                if metadata and "left_width" in metadata
                else 1.0,
                right_width=metadata["right_width"][i]
                if metadata and "right_width" in metadata
                else 1.0,
            )
            self.centerline.append(point)

        self.wall_points = wall_points

        print(f"  - Centerline points: {len(self.centerline)}")
        print(f"  - Wall points: {len(self.wall_points)}")

    def _plan_duct_layout(self) -> List[Dict]:
        """
        Duct 배치 계획 수립

        센터라인을 따라 duct를 배치할 최적의 위치와 길이 결정
        """
        print("\nStep 2: Planning duct layout...")

        duct_plan = []
        total_length = self.centerline[-1].cumulative_distance

        # 기본 duct 길이 계산 (트랙 길이에 비례)
        base_duct_length = min(
            self.max_duct_length, max(self.min_duct_length, total_length / 20)
        )

        current_distance = 0
        duct_id = 0

        while current_distance < total_length:
            # 현재 위치에서의 센터라인 정보 얻기
            cl_point = self._get_centerline_at_distance(current_distance)

            # Duct 길이 결정 (곡률에 따라 조정)
            duct_length = self._calculate_duct_length(cl_point, base_duct_length)

            # Duct 끝점 계산
            end_distance = min(current_distance + duct_length, total_length)
            end_point = self._get_centerline_at_distance(end_distance)

            # Duct 유형 결정 (직선/곡선)
            is_straight = self._is_straight_segment(current_distance, end_distance)

            # Duct 중심선 추출
            duct_centerline = self._extract_duct_centerline(
                current_distance, end_distance
            )

            duct_plan.append(
                {
                    "id": duct_id,
                    "start_distance": current_distance,
                    "end_distance": end_distance,
                    "start_point": np.array([cl_point.x, cl_point.y]),
                    "end_point": np.array([end_point.x, end_point.y]),
                    "centerline": duct_centerline,
                    "is_straight": is_straight,
                    "max_curvature": self._calculate_segment_curvature(
                        current_distance, end_distance
                    ),
                }
            )

            duct_id += 1
            current_distance = end_distance

            # Overlap 방지를 위한 최소 간격
            if current_distance < total_length:
                current_distance += self.duct_thickness * 0.5

        print(f"  - Planned {len(duct_plan)} duct segments")
        print(f"  - Average duct length: {total_length / len(duct_plan):.2f}m")

        return duct_plan

    def _create_duct_segments(self, duct_plan: List[Dict]):
        """Duct 세그먼트 생성"""
        print("\nStep 3: Creating duct segments...")

        self.duct_segments = []

        for plan in duct_plan:
            segment = DuctSegment(
                id=plan["id"],
                start_point=plan["start_point"],
                end_point=plan["end_point"],
                centerline=plan["centerline"],
                assigned_points=[],
                thickness=self.duct_thickness,
                max_curvature=plan["max_curvature"],
                is_straight=plan["is_straight"],
            )

            self.duct_segments.append(segment)

            print(
                f"  - Duct {segment.id}: length={segment.length():.2f}m, "
                f"straight={segment.is_straight}, curvature={segment.max_curvature:.3f}"
            )

    def _assign_points_to_ducts(self):
        """벽 점들을 가장 가까운 duct에 할당"""
        print("\nStep 4: Assigning wall points to ducts...")

        if len(self.wall_points) == 0:
            print("  - No wall points to assign")
            return

        # KD-tree를 사용한 효율적인 점 할당
        wall_kdtree = KDTree(self.wall_points)

        for duct in self.duct_segments:
            # Duct의 바운딩 박스 내 점들 찾기
            bbox = duct.bounding_box()
            margin = self.duct_thickness * 2.0  # 여유 공간

            # 바운딩 박스 + margin 내 점들 쿼리
            indices = wall_kdtree.query_ball_point(
                duct.centerline[len(duct.centerline) // 2],  # 중간점
                duct.length() / 2 + margin,
            )

            if indices:
                duct_points = self.wall_points[indices]

                # Duct 중심선으로부터의 거리 계산
                distances = self._distance_to_duct_centerline(
                    duct_points, duct.centerline
                )

                # Duct 두께 내의 점들만 할당
                valid_mask = distances <= (duct.thickness * 1.5)  # 여유 포함
                duct.assigned_points = duct_points[valid_mask].tolist()

                print(
                    f"  - Duct {duct.id}: assigned {len(duct.assigned_points)} points"
                )

    def _optimize_duct_connections(self):
        """Duct 연결점 최적화"""
        print("\nStep 5: Optimizing duct connections...")

        self.duct_connections = []

        for i in range(len(self.duct_segments) - 1):
            duct_a = self.duct_segments[i]
            duct_b = self.duct_segments[i + 1]

            # 연결점 찾기 (가장 가까운 점)
            connection_point = self._find_connection_point(duct_a, duct_b)

            if connection_point is not None:
                # 연결 각도 차이 계산
                angle_diff = self._calculate_connection_angle(
                    duct_a, duct_b, connection_point
                )

                # 각도 차이가 허용 범위 내인지 확인
                if abs(angle_diff) <= np.radians(45):  # 45도 이내
                    connection = DuctConnection(
                        from_duct=duct_a.id,
                        to_duct=duct_b.id,
                        connection_point=connection_point,
                        angle_diff=angle_diff,
                    )
                    self.duct_connections.append(connection)

                    print(
                        f"  - Connection {duct_a.id}→{duct_b.id}: "
                        f"angle={np.degrees(angle_diff):.1f}°"
                    )

    def _generate_final_walls(self) -> Dict:
        """최종 벽 데이터 생성"""
        print("\nStep 6: Generating final walls...")

        left_walls = []
        right_walls = []

        for duct in self.duct_segments:
            if len(duct.assigned_points) < 2:
                continue

            points = np.array(duct.assigned_points)

            # Duct 중심선 기준 좌우 분리
            left_mask, right_mask = self._separate_left_right(points, duct.centerline)

            if np.any(left_mask):
                left_points = points[left_mask]
                left_curve = self._fit_curve_to_points(left_points)
                left_walls.append(left_curve)

            if np.any(right_mask):
                right_points = points[right_mask]
                right_curve = self._fit_curve_to_points(right_points)
                right_walls.append(right_curve)

        # 모든 곡선 연결
        final_left_wall = self._connect_wall_curves(left_walls)
        final_right_wall = self._connect_wall_curves(right_walls)

        print(f"  - Left wall: {len(final_left_wall)} points")
        print(f"  - Right wall: {len(final_right_wall)} points")

        return {
            "left_wall": final_left_wall,
            "right_wall": final_right_wall,
            "duct_segments": self.duct_segments,
        }

    # ============================================================================
    # 5. 유틸리티 함수들
    # ============================================================================

    def _calculate_cumulative_distance(self, points: np.ndarray, idx: int) -> float:
        """누적 거리 계산"""
        if idx == 0:
            return 0.0
        return np.sum(np.linalg.norm(np.diff(points[: idx + 1], axis=0), axis=1))

    def _get_centerline_at_distance(self, distance: float) -> CenterlinePoint:
        """주어진 거리에서의 센터라인 점 찾기"""
        for point in self.centerline:
            if point.cumulative_distance >= distance:
                return point
        return self.centerline[-1]

    def _calculate_duct_length(
        self, cl_point: CenterlinePoint, base_length: float
    ) -> float:
        """곡률을 고려한 duct 길이 계산"""
        # 곡률이 클수록 짧은 duct (구부리기 어려움)
        curvature_factor = 1.0 / (1.0 + abs(cl_point.curvature) * 2.0)

        # 트랙 너비가 좁을수록 짧은 duct
        total_width = cl_point.left_width + cl_point.right_width
        width_factor = min(1.0, total_width / (self.min_track_width * 2))

        length = base_length * curvature_factor * width_factor

        return np.clip(length, self.min_duct_length, self.max_duct_length)

    def _is_straight_segment(self, start_dist: float, end_dist: float) -> bool:
        """세그먼트가 직선인지 판단"""
        segment_points = []
        for point in self.centerline:
            if start_dist <= point.cumulative_distance <= end_dist:
                segment_points.append([point.x, point.y])

        if len(segment_points) < 3:
            return True

        # PCA로 주성분 분석
        points_array = np.array(segment_points)
        cov_matrix = np.cov(points_array.T)
        eigenvalues = np.linalg.eigvals(cov_matrix)

        # 첫 번째 주성분이 전체 분산의 95% 이상 설명하면 직선으로 판단
        return eigenvalues[0] / eigenvalues.sum() > 0.95

    def _extract_duct_centerline(
        self, start_dist: float, end_dist: float
    ) -> List[np.ndarray]:
        """Duct 중심선 추출"""
        centerline_points = []
        for point in self.centerline:
            if start_dist <= point.cumulative_distance <= end_dist:
                centerline_points.append([point.x, point.y])

        # 보간으로 매끄러운 곡선 생성
        if len(centerline_points) > 3:
            points_array = np.array(centerline_points)
            tck, u = splprep(points_array.T, s=0, k=3)
            u_fine = np.linspace(0, 1, max(20, len(centerline_points)))
            x_fine, y_fine = splev(u_fine, tck)
            return np.column_stack([x_fine, y_fine]).tolist()

        return centerline_points

    def _calculate_segment_curvature(self, start_dist: float, end_dist: float) -> float:
        """세그먼트의 최대 곡률 계산"""
        max_curvature = 0.0
        for point in self.centerline:
            if start_dist <= point.cumulative_distance <= end_dist:
                max_curvature = max(max_curvature, abs(point.curvature))
        return max_curvature

    def _distance_to_duct_centerline(
        self, points: np.ndarray, duct_centerline: List[np.ndarray]
    ) -> np.ndarray:
        """점들에서 duct 중심선까지의 최소 거리 계산"""
        if len(duct_centerline) < 2:
            return np.full(len(points), np.inf)

        duct_array = np.array(duct_centerline)
        distances = []

        for point in points:
            # duct 중심선의 모든 세그먼트에 대해 거리 계산
            seg_distances = []
            for i in range(len(duct_array) - 1):
                dist = self._point_to_segment_distance(
                    point, duct_array[i], duct_array[i + 1]
                )
                seg_distances.append(dist)

            distances.append(min(seg_distances))

        return np.array(distances)

    def _point_to_segment_distance(
        self, point: np.ndarray, seg_start: np.ndarray, seg_end: np.ndarray
    ) -> float:
        """점과 선분 사이의 거리 계산"""
        line_vec = seg_end - seg_start
        point_vec = point - seg_start
        line_len = np.dot(line_vec, line_vec)

        if line_len == 0:
            return np.linalg.norm(point - seg_start)

        t = max(0, min(1, np.dot(point_vec, line_vec) / line_len))
        projection = seg_start + t * line_vec
        return np.linalg.norm(point - projection)

    def _separate_left_right(
        self, points: np.ndarray, duct_centerline: List[np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """점들을 duct 중심선 기준 좌우로 분리"""
        if len(duct_centerline) < 2:
            return np.zeros(len(points), dtype=bool), np.zeros(len(points), dtype=bool)

        # Duct 중심선의 평균 방향 벡터 계산
        duct_array = np.array(duct_centerline)
        avg_direction = np.mean(np.diff(duct_array, axis=0), axis=0)
        avg_direction = avg_direction / np.linalg.norm(avg_direction)

        # 수직 벡터 (왼쪽 기준)
        normal = np.array([-avg_direction[1], avg_direction[0]])

        left_mask = np.zeros(len(points), dtype=bool)
        right_mask = np.zeros(len(points), dtype=bool)

        for i, point in enumerate(points):
            # 가장 가까운 duct 점 찾기
            distances = np.linalg.norm(duct_array - point, axis=1)
            nearest_idx = np.argmin(distances)

            if nearest_idx < len(duct_array):
                duct_point = duct_array[nearest_idx]
                vec_to_point = point - duct_point

                # 내적으로 좌우 판단
                side = np.dot(vec_to_point, normal)

                if side > 0:
                    left_mask[i] = True
                else:
                    right_mask[i] = True

        return left_mask, right_mask

    def _fit_curve_to_points(self, points: np.ndarray) -> np.ndarray:
        """점들에 곡선 피팅"""
        if len(points) < 3:
            return points

        try:
            # 점들을 거리순으로 정렬
            from scipy.spatial.distance import cdist

            start_idx = np.argmin(points[:, 0])  # 가장 왼쪽 점에서 시작
            ordered_points = [points[start_idx]]
            remaining_indices = list(range(len(points)))
            remaining_indices.remove(start_idx)

            # 최근접 이웃으로 순서 정렬
            while remaining_indices:
                last_point = ordered_points[-1]
                distances = cdist([last_point], points[remaining_indices])[0]
                next_idx = remaining_indices[np.argmin(distances)]
                ordered_points.append(points[next_idx])
                remaining_indices.remove(next_idx)

            ordered_array = np.array(ordered_points)

            # B-spline 피팅
            tck, u = splprep(ordered_array.T, s=len(points) * 0.1, k=3)
            u_fine = np.linspace(0, 1, max(50, len(points) // 2))
            x_fine, y_fine = splev(u_fine, tck)

            return np.column_stack([x_fine, y_fine])
        except:
            # 피팅 실패 시 원본 점들 반환
            return points

    def _connect_wall_curves(self, wall_curves: List[np.ndarray]) -> np.ndarray:
        """여러 벽 곡선을 하나로 연결"""
        if not wall_curves:
            return np.array([])

        if len(wall_curves) == 1:
            return wall_curves[0]

        connected = []
        for i, curve in enumerate(wall_curves):
            if i > 0:
                # 이전 곡선의 끝점과 현재 곡선의 시작점 연결
                prev_end = connected[-1]
                curr_start = curve[0]

                # 너무 가까우면 중간점 추가하지 않음
                if np.linalg.norm(prev_end - curr_start) > self.resolution * 2:
                    # 선형 보간으로 연결
                    num_interp = max(
                        3, int(np.linalg.norm(prev_end - curr_start) / self.resolution)
                    )
                    t_values = np.linspace(0, 1, num_interp)[1:-1]  # 끝점 제외
                    for t in t_values:
                        interp_point = prev_end * (1 - t) + curr_start * t
                        connected.append(interp_point)

            connected.extend(curve.tolist())

        return np.array(connected)

    def _find_connection_point(
        self, duct_a: DuctSegment, duct_b: DuctSegment
    ) -> Optional[np.ndarray]:
        """두 duct 사이의 연결점 찾기"""
        # duct_a의 끝점과 duct_b의 시작점 사이의 중점
        if len(duct_a.centerline) > 0 and len(duct_b.centerline) > 0:
            point_a = np.array(duct_a.centerline[-1])
            point_b = np.array(duct_b.centerline[0])

            # 거리가 허용 범위 내인지 확인
            distance = np.linalg.norm(point_a - point_b)
            if distance <= self.connection_tolerance:
                return (point_a + point_b) / 2.0

        return None

    def _calculate_connection_angle(
        self, duct_a: DuctSegment, duct_b: DuctSegment, connection_point: np.ndarray
    ) -> float:
        """두 duct의 연결 각도 차이 계산"""
        if len(duct_a.centerline) < 2 or len(duct_b.centerline) < 2:
            return 0.0

        # duct_a의 끝부분 방향 벡터
        dir_a = np.array(duct_a.centerline[-1]) - np.array(duct_a.centerline[-2])
        dir_a = dir_a / np.linalg.norm(dir_a)

        # duct_b의 시작부분 방향 벡터
        dir_b = np.array(duct_b.centerline[1]) - np.array(duct_b.centerline[0])
        dir_b = dir_b / np.linalg.norm(dir_b)

        # 두 방향 벡터 사이의 각도
        dot_product = np.dot(dir_a, dir_b)
        dot_product = np.clip(dot_product, -1.0, 1.0)
        return np.arccos(dot_product)

    def _validate_walls(self, walls: Dict):
        """생성된 벽 검증"""
        print("\nStep 7: Validating walls...")

        left_wall = walls["left_wall"]
        right_wall = walls["right_wall"]

        # 1. 벽 길이 검증
        if len(left_wall) < 10 or len(right_wall) < 10:
            print("  ⚠️  Warning: Wall curves are too short")

        # 2. 트랙 너비 검증
        if len(left_wall) > 0 and len(right_wall) > 0:
            # 샘플링 점들 사이의 거리 계산
            sample_indices = np.linspace(
                0,
                min(len(left_wall), len(right_wall)) - 1,
                min(20, len(left_wall)),
                dtype=int,
            )

            min_width = float("inf")
            for idx in sample_indices:
                width = np.linalg.norm(left_wall[idx] - right_wall[idx])
                min_width = min(min_width, width)

            print(f"  - Minimum track width: {min_width:.2f}m")

            if min_width < self.min_track_width:
                print(
                    f"  ⚠️  Warning: Track width ({min_width:.2f}m) "
                    f"is below minimum ({self.min_track_width}m)"
                )

    def _create_occupancy_grid(self, walls: Dict) -> np.ndarray:
        """점유격자 생성"""
        print("\nStep 8: Creating occupancy grid...")

        # 전체 공간 크기 계산
        all_points = []
        if len(walls["left_wall"]) > 0:
            all_points.append(walls["left_wall"])
        if len(walls["right_wall"]) > 0:
            all_points.append(walls["right_wall"])

        if not all_points:
            return np.array([])

        all_points_array = np.vstack(all_points)

        x_min, x_max = all_points_array[:, 0].min(), all_points_array[:, 0].max()
        y_min, y_max = all_points_array[:, 1].min(), all_points_array[:, 1].max()

        # 여유 공간 추가
        margin = self.duct_thickness * 2.0
        x_min -= margin
        x_max += margin
        y_min -= margin
        y_max += margin

        # 그리드 크기 계산
        grid_width = int(np.ceil((x_max - x_min) / self.resolution))
        grid_height = int(np.ceil((y_max - y_min) / self.resolution))

        occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

        print(
            f"  - Grid size: {grid_width}x{grid_height} "
            f"({grid_width * self.resolution:.1f}m x {grid_height * self.resolution:.1f}m)"
        )

        # 벽 영역 채우기 (단순화된 구현)
        for wall in [walls["left_wall"], walls["right_wall"]]:
            if len(wall) == 0:
                continue

            for point in wall:
                grid_x = int((point[0] - x_min) / self.resolution)
                grid_y = int((point[1] - y_min) / self.resolution)

                if 0 <= grid_x < grid_width and 0 <= grid_y < grid_height:
                    # duct 두께를 고려한 영역 채우기
                    thickness_pixels = int(
                        np.ceil(self.duct_thickness / self.resolution)
                    )
                    radius = max(1, thickness_pixels // 2)

                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            px = grid_x + dx
                            py = grid_y + dy
                            if 0 <= px < grid_width and 0 <= py < grid_height:
                                if dx * dx + dy * dy <= radius * radius:
                                    occupancy_grid[py, px] = 1

        return occupancy_grid

    def _generate_metadata(self) -> Dict:
        """메타데이터 생성"""
        return {
            "generator": "DuctBasedWallGenerator",
            "version": "1.0",
            "parameters": {
                "duct_thickness": self.duct_thickness,
                "min_duct_length": self.min_duct_length,
                "max_duct_length": self.max_duct_length,
                "max_curvature": self.max_curvature,
                "min_track_width": self.min_track_width,
                "connection_tolerance": self.connection_tolerance,
                "resolution": self.resolution,
            },
            "statistics": {
                "num_ducts": len(self.duct_segments),
                "num_connections": len(self.duct_connections),
                "total_duct_length": sum(d.length() for d in self.duct_segments),
                "avg_duct_length": np.mean([d.length() for d in self.duct_segments])
                if self.duct_segments
                else 0,
            },
        }

    # ============================================================================
    # 6. 시각화 및 디버깅 도구
    # ============================================================================

    def visualize_generation_process(self, walls_result: Dict, save_path: str = None):
        """생성 과정 시각화"""
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(15, 10))

        # 1. 원본 데이터
        ax1 = axes[0, 0]
        centerline_array = np.array([[p.x, p.y] for p in self.centerline])
        ax1.plot(
            centerline_array[:, 0],
            centerline_array[:, 1],
            "b-",
            alpha=0.5,
            label="Centerline",
        )
        ax1.scatter(
            self.wall_points[:, 0],
            self.wall_points[:, 1],
            s=1,
            c="gray",
            alpha=0.3,
            label="Wall points",
        )
        ax1.set_title("1. Input Data")
        ax1.legend()
        ax1.axis("equal")

        # 2. Duct 배치
        ax2 = axes[0, 1]
        ax2.plot(centerline_array[:, 0], centerline_array[:, 1], "b-", alpha=0.3)

        colors = plt.cm.tab20(np.linspace(0, 1, len(self.duct_segments)))
        for duct, color in zip(self.duct_segments, colors):
            duct_array = np.array(duct.centerline)
            ax2.plot(duct_array[:, 0], duct_array[:, 1], "-", color=color, linewidth=2)
            ax2.scatter(
                duct_array[0, 0], duct_array[0, 1], color=color, s=50, marker="o"
            )
            ax2.scatter(
                duct_array[-1, 0], duct_array[-1, 1], color=color, s=50, marker="s"
            )

            # 할당된 점들
            if duct.assigned_points:
                points_array = np.array(duct.assigned_points)
                ax2.scatter(
                    points_array[:, 0], points_array[:, 1], s=10, color=color, alpha=0.5
                )

        ax2.set_title("2. Duct Layout")
        ax2.axis("equal")

        # 3. Duct 연결
        ax3 = axes[0, 2]
        ax3.plot(centerline_array[:, 0], centerline_array[:, 1], "b-", alpha=0.3)

        for duct in self.duct_segments:
            duct_array = np.array(duct.centerline)
            ax3.plot(duct_array[:, 0], duct_array[:, 1], "k-", alpha=0.5)

        for conn in self.duct_connections:
            ax3.scatter(
                conn.connection_point[0],
                conn.connection_point[1],
                s=100,
                c="red",
                marker="*",
                zorder=5,
            )

        ax3.set_title("3. Duct Connections")
        ax3.axis("equal")

        # 4. 최종 벽
        ax4 = axes[1, 0]
        left_wall = walls_result["walls"]["left_wall"]
        right_wall = walls_result["walls"]["right_wall"]

        if len(left_wall) > 0:
            ax4.plot(
                left_wall[:, 0], left_wall[:, 1], "r-", linewidth=2, label="Left wall"
            )
        if len(right_wall) > 0:
            ax4.plot(
                right_wall[:, 0],
                right_wall[:, 1],
                "b-",
                linewidth=2,
                label="Right wall",
            )

        ax4.plot(
            centerline_array[:, 0],
            centerline_array[:, 1],
            "g--",
            alpha=0.5,
            label="Centerline",
        )
        ax4.set_title("4. Final Walls")
        ax4.legend()
        ax4.axis("equal")

        # 5. 점유격자
        ax5 = axes[1, 1]
        occupancy = walls_result["occupancy_grid"]
        if occupancy.size > 0:
            ax5.imshow(
                occupancy,
                cmap="binary",
                origin="lower",
                extent=[
                    0,
                    occupancy.shape[1] * self.resolution,
                    0,
                    occupancy.shape[0] * self.resolution,
                ],
            )
            ax5.set_title("5. Occupancy Grid")
            ax5.set_xlabel("X (m)")
            ax5.set_ylabel("Y (m)")

        # 6. 통계
        ax6 = axes[1, 2]
        ax6.axis("off")

        stats_text = "Generation Statistics:\n\n"
        stats_text += f"Duct Segments: {len(self.duct_segments)}\n"
        stats_text += f"Duct Connections: {len(self.duct_connections)}\n"

        if self.duct_segments:
            total_length = sum(d.length() for d in self.duct_segments)
            avg_length = total_length / len(self.duct_segments)
            stats_text += f"Total Duct Length: {total_length:.1f}m\n"
            stats_text += f"Avg Duct Length: {avg_length:.2f}m\n"

        if (
            "left_wall" in walls_result["walls"]
            and "right_wall" in walls_result["walls"]
        ):
            left_len = len(walls_result["walls"]["left_wall"])
            right_len = len(walls_result["walls"]["right_wall"])
            stats_text += f"\nLeft Wall Points: {left_len}\n"
            stats_text += f"Right Wall Points: {right_len}"

        ax6.text(
            0.1,
            0.5,
            stats_text,
            transform=ax6.transAxes,
            fontfamily="monospace",
            verticalalignment="center",
        )
        ax6.set_title("6. Statistics")

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"\nVisualization saved to: {save_path}")

        plt.show()


# ============================================================================
# 7. 사용 예시
# ============================================================================


def example_usage():
    """사용 예시"""
    # 설정
    config = {
        "duct_thickness": 0.25,  # 15cm 두께의 duct
        "min_duct_length": 10.0,  # 최소 30cm 길이
        "max_duct_length": 2.5,  # 최대 1m 길이
        "max_curvature": 0.3,  # 최대 곡률 0.3 (약 3.3m 반경)
        "min_track_width": 0.8,  # 최소 트랙 너비 80cm
        "connection_tolerance": 0.1,  # 연결 허용 오차 10cm
        "resolution": 0.05,  # 5cm 해상도
    }

    # 생성기 인스턴스화
    generator = DuctBasedWallGenerator(config)

    # 샘플 데이터 생성 (실제로는 centerline.py의 출력 사용)
    import numpy as np

    # 예시 센터라인 (원형)
    theta = np.linspace(0, 2 * np.pi, 100)
    centerline = np.column_stack([5 + 3 * np.cos(theta), 5 + 2 * np.sin(theta)])

    # 예시 벽 점들 (센터라인 주변에 무작위 분포)
    np.random.seed(42)
    n_wall_points = 500
    wall_points = []
    for t in theta:
        # 중심점
        cx = 5 + 3 * np.cos(t)
        cy = 5 + 2 * np.sin(t)

        # 수직 방향
        nx = -3 * np.sin(t)
        ny = 2 * np.cos(t)
        normal = np.array([-ny, nx])  # 수직 벡터
        normal = normal / np.linalg.norm(normal)

        # 좌우로 점 추가
        for side in [-1, 1]:  # -1: 왼쪽, 1: 오른쪽
            offset = 1.0 + 0.3 * np.random.randn()  # 평균 1m 너비
            point = np.array([cx, cy]) + side * offset * normal
            point += 0.1 * np.random.randn(2)  # 노이즈 추가
            wall_points.append(point)

    wall_points = np.array(wall_points)

    # 메타데이터 (예시)
    metadata = {
        "curvature": np.sin(theta * 2) * 0.2,  # 가변 곡률
        "left_width": np.ones(100) * 1.2,  # 왼쪽 너비 1.2m
        "right_width": np.ones(100) * 0.8,  # 오른쪽 너비 0.8m
    }

    # 벽 생성 실행
    result = generator.generate_walls(centerline, wall_points, metadata)

    # 결과 시각화
    generator.visualize_generation_process(result, save_path="duct_generation.png")

    return result


if __name__ == "__main__":
    result = example_usage()
    print("\n" + "=" * 60)
    print("GENERATION COMPLETED SUCCESSFULLY!")
    print("=" * 60)
