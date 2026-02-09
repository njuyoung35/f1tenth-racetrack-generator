import os

import matplotlib.pyplot as plt
import numpy as np
from concave_hull import concave_hull, concave_hull_indexes
from scipy import interpolate
from scipy.spatial import Delaunay, Voronoi, voronoi_plot_2d
from shapely.geometry import Polygon

from lloyd import Field

RANDOM_POINT_MARGIN = 0.25
CONCAVITY = 0.33


class CenterlineGenerator:
    def __init__(self, config):
        self.width = config.get("width", 20.0)
        self.height = config.get("height", 10.0)
        self.n_points = config.get("n_points", 60)

        self.max_lloyd_iterations = config.get("max_lloyd_iterations", 10)
        self.max_verifying_iterations = config.get("max_verifying_iterations", 50)

        self.virtual_grid_width = config.get("virtual_grid_width", 5.0)
        self.virtual_grid_coverage = config.get("virtual_grid_coverage", 0.8)

        self.curvature_threshold = config.get("curvature_threshold", 0.3)

        self.visualize = config.get("visualize", False)
        self.debug_output_dir = config.get("debug_output_dir", "debug")

    def generate_points(self) -> np.ndarray:
        """
        초기 랜덤 포인트 생성

        Returns:
            np.ndarray: (n_points, 2) 형태의 포인트 배열
        """
        # 경계에 여유 공간 추가
        margin = RANDOM_POINT_MARGIN
        points = np.random.rand(self.n_points, 2)
        points[:, 0] = points[:, 0] * (self.width - 2 * margin) + margin
        points[:, 1] = points[:, 1] * (self.height - 2 * margin) + margin

        if self.visualize:
            self._visualize_points(points)

        return points

    def run_lloyds_algorithm(self, points) -> Voronoi:
        """
        Lloyd 알고리즘 실행

        Args:
            points: 초기 포인트 배열

        Returns:
            Voronoi
        """

        field = Field(points)
        voronoi_plot_2d(
            field.voronoi,
            show_vertices=False,
            line_colors="orange",
            line_width=2,
            line_alpha=0.6,
            point_size=2,
        )
        plt.savefig(os.path.join(self.debug_output_dir, "01_initial_voronoi.png"))

        for iteration in range(self.max_lloyd_iterations):
            field.relax()
            voronoi_plot_2d(
                field.voronoi,
                show_vertices=False,
                line_colors="orange",
                line_width=2,
                line_alpha=0.6,
                point_size=2,
            )
            plt.savefig(
                os.path.join(
                    self.debug_output_dir, f"02_lloyd_iteration_{iteration + 1:02d}.png"
                )
            )

        return field.voronoi

    def select_regions_from_virtual_grid(self, vor: Voronoi) -> list:
        """
        각 셀에서 상위 K개의 region 중 무작위로 하나 선택

        Args:
            vor: Voronoi 다이어그램

        Returns:
            list: 선택된 region 인덱스 리스트
        """
        # 가상 그리드 크기 계산
        grid_cols = int(np.ceil(self.width / self.virtual_grid_width))
        grid_rows = int(np.ceil(self.height / self.virtual_grid_width))

        # 각 셀당 고려할 상위 region 수
        k_options = [3, 4, 5]  # 다양성을 위해 K값을 랜덤하게 선택

        selected_regions = []

        for i in range(grid_cols):
            for j in range(grid_rows):
                # coverage 비율에 따라 일부 셀만 선택
                if np.random.random() > self.virtual_grid_coverage:
                    continue

                # 셀 중심 계산
                cell_center_x = (
                    i * self.virtual_grid_width + self.virtual_grid_width / 2
                )
                cell_center_y = (
                    j * self.virtual_grid_width + self.virtual_grid_width / 2
                )

                # 경계 체크
                if cell_center_x > self.width or cell_center_y > self.height:
                    continue

                # 현재 셀에 대한 K값 선택 (다양성을 위해)
                k = np.random.choice(k_options)

                # 모든 Voronoi point와의 거리 계산
                distances = np.linalg.norm(
                    vor.points - np.array([cell_center_x, cell_center_y]), axis=1
                )

                # 상위 K개의 region 중에서 선택
                top_k_indices = np.argsort(distances)[:k]

                # 거리 가중치 계산 (가까울수록 높은 가중치)
                top_k_distances = distances[top_k_indices]
                weights = 1.0 / (top_k_distances + 0.1)  # 0으로 나누기 방지
                weights = weights / weights.sum()  # 정규화

                # 가중치 기반으로 하나 선택
                chosen_idx = np.random.choice(top_k_indices, p=weights)
                selected_regions.append(chosen_idx)

        # 중복 제거
        selected_regions = list(set(selected_regions))

        # 최소 region 수 보장
        if len(selected_regions) < 3:
            # 부족하면 추가로 무작위 선택
            all_regions = list(range(len(vor.points)))
            additional_needed = 3 - len(selected_regions)
            additional_regions = np.random.choice(
                list(set(all_regions) - set(selected_regions)),
                size=min(additional_needed, len(all_regions) - len(selected_regions)),
                replace=False,
            )
            selected_regions.extend(additional_regions)

        print(
            f"Selected {len(selected_regions)} regions from {grid_cols * grid_rows} cells"
        )

        if self.visualize:
            self._visualize_grid_selection(vor, selected_regions, grid_cols, grid_rows)

        return selected_regions

    def create_concave_hull_polygon(
        self, vor: Voronoi, selected_region_indices: list
    ) -> np.ndarray:
        """
        선택된 region들의 원래 점(seed points)을 사용하여 오목 껍질(concave hull) 생성
        """
        # 선택된 region들의 원래 점(seed points) 사용
        selected_points = []
        for idx in selected_region_indices:
            # Voronoi diagram의 원래 점을 직접 사용
            point = vor.points[idx]
            selected_points.append(point)

        if len(selected_points) < 3:
            raise ValueError(
                f"Too few points ({len(selected_points)}) for concave hull"
            )

        points_array = np.array(selected_points)

        # concavity 값을 적절히 조정 (2.0 → 10.0 사이에서 실험)
        concave_hull_points = concave_hull(
            points_array,
            concavity=CONCAVITY,
            length_threshold=0.0,
        )

        # 폐루프를 위해 첫 점을 끝에 추가
        if len(concave_hull_points) > 0:
            if not np.array_equal(concave_hull_points[0], concave_hull_points[-1]):
                concave_hull_points = np.vstack(
                    [concave_hull_points, concave_hull_points[0]]
                )

        if self.visualize:
            self._visualize_concave_hull(points_array, concave_hull_points)

        return concave_hull_points

    def interpolate_polygon_with_voronoi_vertices(
        self, vor: Voronoi, polygon: np.ndarray
    ) -> np.ndarray:
        """
        폴리곤을 Voronoi vertices를 사용하여 보간하고 곡률 제한 검사

        Args:
            vor: Voronoi 다이어그램
            polygon: 보간할 폴리곤

        Returns:
            np.ndarray: 보간된 중심선 포인트
        """
        # 폴리곤의 각 변에 대해 Voronoi vertices 선택
        interpolated_points = []

        for i in range(len(polygon) - 1):
            start_point = polygon[i]
            end_point = polygon[i + 1]

            # 두 점 사이의 선분 근처에 있는 Voronoi vertices 찾기
            segment_length = np.linalg.norm(end_point - start_point)
            max_distance = segment_length * 0.3  # 선분에서 최대 거리

            # 선분 근처의 vertices 필터링
            nearby_vertices = []
            for vertex in vor.vertices:
                # 점과 선분 사이의 거리 계산
                distance = self._point_to_line_distance(vertex, start_point, end_point)
                if distance < max_distance:
                    # 선분의 경계 내에 있는지 체크
                    if self._is_point_between(
                        vertex, start_point, end_point, margin=1.0
                    ):
                        nearby_vertices.append(vertex)

            # 인접한 vertices 선택 (너무 많으면 곡률 문제 발생 가능)
            max_vertices_per_segment = max(
                3, int(segment_length / self.virtual_grid_width * 2)
            )
            if len(nearby_vertices) > max_vertices_per_segment:
                # 선분과의 거리 기준으로 정렬하여 가까운 것 선택
                distances = [
                    self._point_to_line_distance(v, start_point, end_point)
                    for v in nearby_vertices
                ]
                sorted_indices = np.argsort(distances)
                nearby_vertices = [
                    nearby_vertices[i]
                    for i in sorted_indices[:max_vertices_per_segment]
                ]

            # 시작점 추가
            interpolated_points.append(start_point)

            # 근처 vertices 추가 (거리순으로 정렬)
            if nearby_vertices:
                # 시작점 기준으로 정렬
                sorted_vertices = sorted(
                    nearby_vertices, key=lambda v: np.linalg.norm(v - start_point)
                )
                interpolated_points.extend(sorted_vertices)

        # 마지막 점 추가
        interpolated_points.append(polygon[-1])

        interpolated_points = np.array(interpolated_points)

        # 곡률 검사 및 보간
        return self._refine_with_curvature_check(interpolated_points)

    def _refine_with_curvature_check(self, points: np.ndarray) -> np.ndarray:
        """
        곡률 제한을 고려하여 포인트 리파인

        Args:
            points: 초기 포인트 배열

        Returns:
            np.ndarray: 리파인된 포인트 배열
        """
        max_iterations = 10

        for iteration in range(max_iterations):
            # 스플라인 보간
            try:
                # 닫힌 곡선으로 처리
                tck, u = interpolate.splprep(
                    [points[:, 0], points[:, 1]],
                    s=0,
                    per=True,  # 주기적(폐곡선)
                )

                # 더 조밀한 포인트로 평가
                u_fine = np.linspace(0, 1, len(points) * 10)
                x_fine, y_fine = interpolate.splev(u_fine, tck)

                # 곡률 계산
                curvature = self._calculate_spline_curvature(tck, u_fine)
                max_curvature = np.max(np.abs(curvature))

                if max_curvature <= self.curvature_threshold:
                    # 곡률이 임계값 이하일 경우 보간 결과 반환
                    centerline = np.column_stack([x_fine, y_fine])

                    if self.visualize:
                        self._visualize_curvature_check(
                            points, centerline, curvature, iteration
                        )

                    return centerline
                else:
                    # 곡률이 높은 부분 찾기
                    high_curvature_indices = np.where(
                        np.abs(curvature) > self.curvature_threshold
                    )[0]
                    if len(high_curvature_indices) > 0:
                        # 가장 높은 곡률의 위치 찾기
                        peak_idx = high_curvature_indices[
                            np.argmax(np.abs(curvature[high_curvature_indices]))
                        ]
                        peak_param = u_fine[peak_idx]

                        # 해당 파라미터에 가장 가까운 원본 포인트 찾기
                        distances = np.abs(u - peak_param)
                        remove_idx = np.argmin(distances)

                        # 곡률이 높은 포인트 제거
                        if len(points) > 10:  # 최소 포인트 수 유지
                            points = np.delete(points, remove_idx, axis=0)
                        else:
                            break
            except Exception as e:
                print(f"Curvature refinement iteration {iteration} failed: {e}")
                break

        # 최대 반복 후에도 만족스럽지 않으면 현재 포인트 반환
        return points

    def _calculate_spline_curvature(self, tck, u_values):
        """스플라인의 곡률 계산"""
        dx, dy = interpolate.splev(u_values, tck, der=1)
        ddx, ddy = interpolate.splev(u_values, tck, der=2)

        curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
        return np.nan_to_num(curvature, nan=0.0, posinf=0.0, neginf=0.0)

    def _point_to_line_distance(self, point, line_start, line_end):
        """점과 선분 사이의 거리 계산"""
        line_vec = line_end - line_start
        point_vec = point - line_start
        line_length = np.dot(line_vec, line_vec)

        if line_length == 0:
            return np.linalg.norm(point - line_start)

        t = max(0, min(1, np.dot(point_vec, line_vec) / line_length))
        projection = line_start + t * line_vec
        return np.linalg.norm(point - projection)

    def _is_point_between(self, point, line_start, line_end, margin=0.5):
        """점이 선분의 연장선상에 있는지 확인 (margin으로 여유 부여)"""
        line_vec = line_end - line_start
        point_vec = point - line_start

        # 스칼라 투영
        t = np.dot(point_vec, line_vec) / np.dot(line_vec, line_vec)

        # margin만큼의 여유를 둠
        return -margin <= t <= (1 + margin)

    def _calculate_curvature(self, path: np.ndarray) -> np.ndarray:
        """
        경로의 곡률 계산

        Args:
            path: 경로 포인트 배열

        Returns:
            np.ndarray: 각 점에서의 곡률
        """
        if len(path) < 3:
            return np.zeros(len(path))

        dx = np.gradient(path[:, 0])
        dy = np.gradient(path[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)

        curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
        curvature = np.nan_to_num(curvature, nan=0.0, posinf=0.0, neginf=0.0)

        return curvature

    def _calculate_track_length(self, path: np.ndarray) -> float:
        """트랙 길이 계산"""
        return np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))

    def generate(self) -> np.ndarray:
        """
        센터라인 생성 메인 메서드

        Returns:
            np.ndarray: 생성된 센터라인
        """
        for attempt in range(self.max_verifying_iterations):
            print(f"Attempt {attempt + 1}/{self.max_verifying_iterations}")

            # 1. 초기 포인트 생성
            points = self.generate_points()

            # 2. Lloyd 알고리즘 실행
            vor = self.run_lloyds_algorithm(points)

            # 3. 가상 그리드에서 region 선택
            selected_regions = self.select_regions_from_virtual_grid(vor)

            # 4. 선택된 region들의 중심점으로 오목 껍질 생성
            concave_polygon = self.create_concave_hull_polygon(vor, selected_regions)

            # 5. 폴리곤을 Voronoi vertices로 보간 및 곡률 검사
            centerline = self.interpolate_polygon_with_voronoi_vertices(
                vor, concave_polygon
            )

            # 6. 최종 보간 및 메타데이터 생성
            centerline = self._final_interpolation(centerline)

            # 검증
            polygon = Polygon(centerline)
            if not polygon.is_valid:
                print("Invalid centerline generated, retrying...")
                continue

            # if self.validate_centerline(centerline):
            print("Valid centerline generated!")
            return centerline

            # print("Validation failed, retrying...")

        raise RuntimeError(
            "Failed to generate valid centerline within maximum attempts"
        )

    def _final_interpolation(self, points: np.ndarray) -> np.ndarray:
        """최종 보간"""
        # 스플라인 보간으로 매끄러운 곡선 생성
        try:
            tck, u = interpolate.splprep([points[:, 0], points[:, 1]], s=0, per=True)

            # 트랙 길이에 기반한 보간 포인트 수 결정
            track_length = self._calculate_track_length(points)
            n_interp_points = max(100, int(track_length * 2))  # 0.5m 간격 정도
            # 임시로 추가해놨으니 꼭 수정할 것!
            n_interp_points *= 10

            u_fine = np.linspace(0, 1, n_interp_points)
            x_fine, y_fine = interpolate.splev(u_fine, tck)

            centerline = np.column_stack([x_fine, y_fine])

            if self.visualize:
                self._visualize_final_centerline(points, centerline)

            return centerline
        except:
            # 보간 실패 시 원본 포인트 반환
            return points

    # 시각화 메서드
    def _visualize_points(self, points: np.ndarray):
        """초기 랜덤 포인트 시각화"""
        plt.figure(figsize=(12, 8))
        plt.scatter(points[:, 0], points[:, 1], s=10, c="blue", alpha=0.6)
        plt.plot(
            [0, self.width, self.width, 0, 0],
            [0, 0, self.height, self.height, 0],
            "r-",
            linewidth=2,
        )
        plt.title("Initial Random Points")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.grid(True, alpha=0.3)
        plt.axis("equal")
        plt.savefig(os.path.join(self.debug_output_dir, "00_initial_random_points.png"))
        plt.close()

    def _visualize_grid_selection(self, vor, selected_indices, grid_cols, grid_rows):
        """그리드 선택 시각화"""
        plt.figure(figsize=(12, 8))

        # 경계 영역 표시
        plt.plot(
            [0, self.width, self.width, 0, 0],
            [0, 0, self.height, self.height, 0],
            "k-",
            linewidth=2,
            label="Boundary",
        )

        # Voronoi 다이어그램 그리기 (경계 내부만)
        # Voronoi points가 [-width, -height] 범위에 있으므로 변환 필요
        # 또는 실제로 사용된 points를 확인
        actual_points = vor.points[: self.n_points]  # Lloyd 알고리즘 후의 points

        # Voronoi edges 그리기
        for simplex in vor.ridge_vertices:
            if -1 not in simplex:  # 무한대 edge 제외
                p1, p2 = vor.vertices[simplex[0]], vor.vertices[simplex[1]]
                # 경계 내부의 edge만 그리기
                if (
                    0 <= p1[0] <= self.width
                    and 0 <= p1[1] <= self.height
                    and 0 <= p2[0] <= self.width
                    and 0 <= p2[1] <= self.height
                ):
                    plt.plot(
                        [p1[0], p2[0]], [p1[1], p2[1]], "orange", linewidth=1, alpha=0.3
                    )

        # Voronoi vertices 표시
        valid_vertices = []
        for vertex in vor.vertices:
            if 0 <= vertex[0] <= self.width and 0 <= vertex[1] <= self.height:
                valid_vertices.append(vertex)

        if valid_vertices:
            valid_vertices = np.array(valid_vertices)
            plt.scatter(
                valid_vertices[:, 0],
                valid_vertices[:, 1],
                s=10,
                c="orange",
                alpha=0.5,
                label="Voronoi Vertices",
            )

        # 가상 그리드 그리기
        for i in range(grid_cols + 1):
            x = i * self.virtual_grid_width
            plt.axvline(x=x, color="gray", linestyle="--", alpha=0.5)

        for j in range(grid_rows + 1):
            y = j * self.virtual_grid_width
            plt.axhline(y=y, color="gray", linestyle="--", alpha=0.5)

        # 선택된 region들 표시 (실제 points 사용)
        if len(actual_points) > 0:
            # selected_indices가 valid한지 확인
            valid_selected = [
                idx for idx in selected_indices if idx < len(actual_points)
            ]
            if valid_selected:
                selected_points = actual_points[valid_selected]
                plt.scatter(
                    selected_points[:, 0],
                    selected_points[:, 1],
                    s=100,
                    c="red",
                    marker="x",
                    linewidths=2,
                    label="Selected Regions",
                )

        plt.title("Virtual Grid Region Selection")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xlim(-1, self.width + 1)  # 약간의 여유 공간
        plt.ylim(-1, self.height + 1)
        plt.savefig(os.path.join(self.debug_output_dir, "03_grid_selection.png"))
        plt.close()

    def _visualize_concave_hull(self, centroids, hull_points):
        """오목 껍질 시각화"""
        plt.figure(figsize=(12, 8))

        # 중심점 표시
        plt.scatter(
            centroids[:, 0],
            centroids[:, 1],
            s=50,
            c="blue",
            alpha=0.6,
            label="Centroids",
        )

        # 오목 껍질 표시
        plt.plot(
            hull_points[:, 0],
            hull_points[:, 1],
            "r-",
            linewidth=2,
            label="Concave Hull",
        )

        plt.scatter(hull_points[:, 0], hull_points[:, 1], s=30, c="red", marker="o")

        plt.title("Concave Hull from Selected Centroids")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis("equal")
        plt.savefig(os.path.join(self.debug_output_dir, "04_concave_hull.png"))
        plt.close()

    def _visualize_curvature_check(
        self, original_points, centerline, curvature, iteration
    ):
        """곡률 검사 시각화"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))

        # 왼쪽: 포인트와 중심선
        ax1.scatter(
            original_points[:, 0],
            original_points[:, 1],
            s=20,
            c="blue",
            alpha=0.6,
            label="Original Points",
        )
        ax1.plot(
            centerline[:, 0], centerline[:, 1], "r-", linewidth=2, label="Centerline"
        )
        ax1.set_title(f"Centerline - Iteration {iteration}")
        ax1.set_xlabel("X (m)")
        ax1.set_ylabel("Y (m)")
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis("equal")

        # 오른쪽: 곡률 분포
        ax2.plot(curvature, "b-", linewidth=1)
        ax2.axhline(
            y=0.3, color="r", linestyle="--", alpha=0.7, label="Curvature Threshold"
        )
        ax2.axhline(y=-0.3, color="r", linestyle="--", alpha=0.7)
        ax2.fill_between(range(len(curvature)), -0.3, 0.3, alpha=0.1, color="green")
        ax2.set_title("Curvature Distribution")
        ax2.set_xlabel("Point Index")
        ax2.set_ylabel("Curvature")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(
            os.path.join(
                self.debug_output_dir, f"05_curvature_check_{iteration:02d}.png"
            )
        )
        plt.close()

    def _visualize_final_centerline(self, original_points, centerline):
        """최종 중심선 시각화"""
        plt.figure(figsize=(12, 8))

        # 원본 포인트
        plt.scatter(
            original_points[:, 0],
            original_points[:, 1],
            s=20,
            c="blue",
            alpha=0.3,
            label="Original Points",
        )

        # 최종 중심선
        plt.plot(
            centerline[:, 0],
            centerline[:, 1],
            "r-",
            linewidth=2,
            label="Final Centerline",
        )

        plt.title("Final Centerline")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis("equal")
        plt.savefig(os.path.join(self.debug_output_dir, "06_final_centerline.png"))
        plt.close()
