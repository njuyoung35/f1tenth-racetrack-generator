./README.md
````
# f1tenth-racetrack-generator

## Table of Contents

- 1. Introduction and Aims
- 2. System requirements
- 3. Input parameters
- 4. Algorithms explained
  - 4.1. centerline generation
  - 4.2. wall generation
  - 4.3. global path optimization
- 5. Output format
- 6. Example and usage
- 7. Troubleshooting?
- 8. References

## 1. Introduction and Aims

f1tenth-racetrack-generator는 f1tenth 자율주행 레이싱 대회를 위한 레이스트랙을 자동으로 생성하는 도구입니다. 실제 f1tenth 트랙은 구부러지는 duct(도관)를 사용하여 구성되며, 이 도구는 다음과 같은 특징을 가진 트랙을 생성합니다:
    
- **실제 F1Tenth 트랙 특성 반영**: 실제 F1Tenth 레이스에서 사용되는 구부러지는 덕트(duct)를 벽으로 사용하는 특성을 반영
- **공간 분할 방식**: 주어진 직사각형 공간을 분할하여 트랙 구성 (일반 F1 트랙과 달리)
- **단일 루프 구조**: 갈림길 없이 단일 폐루프 트랙 생성
- **막다른 길 최소화**: 미로나 뇌 주름과 같은 형태이지만 완전한 막다른 길은 없음
- **물리적 벽 공유**: 좌/우 벽이 분리된 물리적 장애물이 아닐 수 있으며, 하나의 덕트가 여러 부분의 벽을 담당할 수 있음

## 2. System requirements

- uv (>= 0.9.17)

## 3. Input parameters

입력 파라미터는 다음과 같습니다.

## 4. Algorithms explained

### 4.1 centerline generation

(almost completed)

- 1. generate initial random points (boundary from `width`, `height`, and number of points from `n_points`)
- 2. run Lloyd's algorithm (`max_lloyd_iterations` times) to pseudo-normalizing the spacing between points.
- 3. select random region indices from generated voronoi diagram by virtual_grid. (with coverage `virtual_grid_coverage`)
- 4. get concave hull forming polygon that connects given regions' centroids.
- 5. based on polygon, select interpolating vertex from voronoi diagram, considering `curvature_threshold` and non-crossing validation.
- 6. with given polygon, interpolate it, make it centerline

### 4.2 wall generation

(need to be updated)

- 1. with given centerline, get metadata for each points (parameter t, tangent vector, curvature, distance along curve, left raycast length, right raycast length)
  - raycast length mean, try to find first raycasted point from its polygon, where raycast origin point is the given point, and direction is perpendicular to its tangent vector, so left and right.
- 2. given custom lateral offset function with function signature contains parameters as mentioned above, generate point clouds for each points of centerline.
- 3. given point cloud, generate wall curves with DBSCAN
- 4. give duct thickness to wall curves.
- 5. summary and get metadata of track. (generated centerline (each point includes coordinates, cumulative distance along centerline for Frenet coordinates, curvature, left width, right width), wall occupancy grid, (TODO:optimzed global path))


### 4.3 global path optimization

## 5. Output format

(todo)

- `centerline.npy` : centerline.py에서 생성된 센터라인 포인트 (x, y, t, cumulative_distance, curvature, left_width, right_width) 배열 (N x 7)
- `wall.npy` : wall.py에서 생성된 벽 occupancy grid (w x h) (resolution 고려)
- `optimized_global_path.npy` : optimized_global_path.py에서 생성된 최적화된 전체 경로 (x, y, t, curvature, left_width, right_width) 배열 (N x 7)

## 6. Example and usage

```bash
# uv sync

# run
uv run python main.py
```

## 7. Troubleshooting?

## 8. References

- https://github.com/mvanlobensels/random-track-generator
- https://en.wikipedia.org/wiki/Voronoi_diagram
- https://en.wikipedia.org/wiki/Lloyd%27s_algorithm
- https://en.wikipedia.org/wiki/DBSCAN
- assisted by llm: deepseek
- https://github.com/duhaime/lloyd/tree/master
- https://github.com/cubao/concave_hull

````
./__init__.py
```python

```
./centerline.py
```python
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

```
./context.md
`````
./README.md
````
# f1tenth-racetrack-generator

## Table of Contents

- 1. Introduction and Aims
- 2. System requirements
- 3. Input parameters
- 4. Algorithms explained
  - 4.1. centerline generation
  - 4.2. wall generation
  - 4.3. global path optimization
- 5. Output format
- 6. Example and usage
- 7. Troubleshooting?
- 8. References

## 1. Introduction and Aims

f1tenth-racetrack-generator는 f1tenth 자율주행 레이싱 대회를 위한 레이스트랙을 자동으로 생성하는 도구입니다. 실제 f1tenth 트랙은 구부러지는 duct(도관)를 사용하여 구성되며, 이 도구는 다음과 같은 특징을 가진 트랙을 생성합니다:
    
- **실제 F1Tenth 트랙 특성 반영**: 실제 F1Tenth 레이스에서 사용되는 구부러지는 덕트(duct)를 벽으로 사용하는 특성을 반영
- **공간 분할 방식**: 주어진 직사각형 공간을 분할하여 트랙 구성 (일반 F1 트랙과 달리)
- **단일 루프 구조**: 갈림길 없이 단일 폐루프 트랙 생성
- **막다른 길 최소화**: 미로나 뇌 주름과 같은 형태이지만 완전한 막다른 길은 없음
- **물리적 벽 공유**: 좌/우 벽이 분리된 물리적 장애물이 아닐 수 있으며, 하나의 덕트가 여러 부분의 벽을 담당할 수 있음

## 2. System requirements

- uv (>= 0.9.17)

## 3. Input parameters

입력 파라미터는 다음과 같습니다.

## 4. Algorithms explained

### 4.1 centerline generation

(almost completed)

- 1. generate initial random points (boundary from `width`, `height`, and number of points from `n_points`)
- 2. run Lloyd's algorithm (`max_lloyd_iterations` times) to pseudo-normalizing the spacing between points.
- 3. select random region indices from generated voronoi diagram by virtual_grid. (with coverage `virtual_grid_coverage`)
- 4. get concave hull forming polygon that connects given regions' centroids.
- 5. based on polygon, select interpolating vertex from voronoi diagram, considering `curvature_threshold` and non-crossing validation.
- 6. with given polygon, interpolate it, make it centerline

### 4.2 wall generation

(need to be updated)

- 1. with given centerline, get metadata for each points (parameter t, tangent vector, curvature, distance along curve, left raycast length, right raycast length)
  - raycast length mean, try to find first raycasted point from its polygon, where raycast origin point is the given point, and direction is perpendicular to its tangent vector, so left and right.
- 2. given custom lateral offset function with function signature contains parameters as mentioned above, generate point clouds for each points of centerline.
- 3. given point cloud, generate wall curves with DBSCAN
- 4. give duct thickness to wall curves.
- 5. summary and get metadata of track. (generated centerline (each point includes coordinates, cumulative distance along centerline for Frenet coordinates, curvature, left width, right width), wall occupancy grid, (TODO:optimzed global path))


### 4.3 global path optimization

## 5. Output format

(todo)

- `centerline.npy` : centerline.py에서 생성된 센터라인 포인트 (x, y, t, cumulative_distance, curvature, left_width, right_width) 배열 (N x 7)
- `wall.npy` : wall.py에서 생성된 벽 occupancy grid (w x h) (resolution 고려)
- `optimized_global_path.npy` : optimized_global_path.py에서 생성된 최적화된 전체 경로 (x, y, t, curvature, left_width, right_width) 배열 (N x 7)

## 6. Example and usage

```bash
# uv sync

# run
uv run python main.py
```

## 7. Troubleshooting?

## 8. References

- https://github.com/mvanlobensels/random-track-generator
- https://en.wikipedia.org/wiki/Voronoi_diagram
- https://en.wikipedia.org/wiki/Lloyd%27s_algorithm
- https://en.wikipedia.org/wiki/DBSCAN
- assisted by llm: deepseek
- https://github.com/duhaime/lloyd/tree/master
- https://github.com/cubao/concave_hull

````
./__init__.py
```python

```
./centerline.py
```python
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

`````
./duct.py
```python

```
./lloyd.py
```python
# copy & pasted from:
# https://github.com/duhaime/lloyd/blob/master/lloyd/lloyd.py
# origin author: duhaime

import numpy as np
from scipy.spatial import Voronoi


class Field:
    """
    Create a Voronoi map that can be used to run Lloyd
    relaxation on an array of 2D points. For background,
    see: https://en.wikipedia.org/wiki/Lloyd%27s_algorithm
    """

    def __init__(self, *args, **kwargs):
        """
        Store the points and bounding box of the points to which
        Lloyd relaxation will be applied.
        @param np.array `arr`: a numpy array with shape n, 2, where n
          is the number of 2D points to be moved
        @param float `epsilon`: the delta between the input point
          domain and the pseudo-points used to constrain the points
        """
        arr = args[0]
        if not isinstance(arr, np.ndarray) or arr.shape[1] != 2:
            raise Exception("Please provide a numpy array with shape n,2")
        self.points = arr
        # find the bounding box of the input data
        self.domains = self.get_domains(arr)
        # ensure no two points have the exact same coords
        self.jitter_points()
        self.bb_points = self.get_bb_points(arr)
        self.constrain = kwargs.get("constrain", True)
        self.build_voronoi()

    def jitter_points(self, scalar=0.000000001):
        """
        Ensure no two points have the same coords or else the number
        of regions will be less than the number of input points
        """
        while self.points_contain_duplicates():
            positive = np.random.rand(len(self.points), 2) * scalar
            negative = np.random.rand(len(self.points), 2) * scalar
            self.points = self.points + positive - negative
            self.constrain_points()

    def constrain_points(self):
        """
        Update any points that have drifted beyond the boundaries of this space
        """
        for point in self.points:
            if point[0] < self.domains["x"]["min"]:
                point[0] = self.domains["x"]["min"]
            if point[0] > self.domains["x"]["max"]:
                point[0] = self.domains["x"]["max"]
            if point[1] < self.domains["y"]["min"]:
                point[1] = self.domains["y"]["min"]
            if point[1] > self.domains["y"]["max"]:
                point[1] = self.domains["y"]["max"]

    def get_domains(self, arr):
        """
        Return an object with the x, y domains of `arr`
        """
        x = arr[:, 0]
        y = arr[:, 1]
        return {
            "x": {
                "min": min(x),
                "max": max(x),
            },
            "y": {
                "min": min(y),
                "max": max(y),
            },
        }

    def get_bb_points(self, arr):
        """
        Given an array of 2D points, return the four vertex bounding box
        """
        return np.array(
            [
                [self.domains["x"]["min"], self.domains["y"]["min"]],
                [self.domains["x"]["max"], self.domains["y"]["min"]],
                [self.domains["x"]["min"], self.domains["y"]["max"]],
                [self.domains["x"]["max"], self.domains["y"]["max"]],
            ]
        )

    def build_voronoi(self):
        """
        Build a voronoi map from self.points. For background on
        self.voronoi attributes, see: https://docs.scipy.org/doc/scipy/
          reference/generated/scipy.spatial.Voronoi.html
        """
        # build the voronoi tessellation map
        self.voronoi = Voronoi(self.points, qhull_options="Qbb Qc Qx")

        # constrain voronoi vertices within bounding box
        if self.constrain:
            for idx, vertex in enumerate(self.voronoi.vertices):
                x, y = vertex
                if x < self.domains["x"]["min"]:
                    self.voronoi.vertices[idx][0] = self.domains["x"]["min"]
                if x > self.domains["x"]["max"]:
                    self.voronoi.vertices[idx][0] = self.domains["x"]["max"]
                if y < self.domains["y"]["min"]:
                    self.voronoi.vertices[idx][1] = self.domains["y"]["min"]
                if y > self.domains["y"]["max"]:
                    self.voronoi.vertices[idx][1] = self.domains["y"]["max"]

    def points_contain_duplicates(self):
        """
        Return a boolean indicating whether self.points contains duplicates
        """
        vals, count = np.unique(self.points, return_counts=True)
        return np.any(vals[count > 1])

    def find_centroid(self, vertices):
        """
        Find the centroid of a Voroni region described by `vertices`,
        and return a np array with the x and y coords of that centroid.
        The equation for the method used here to find the centroid of a
        2D polygon is given here: https://en.wikipedia.org/wiki/
          Centroid#Of_a_polygon
        @params: np.array `vertices` a numpy array with shape n,2
        @returns np.array a numpy array that defines the x, y coords
          of the centroid described by `vertices`
        """
        area = 0
        centroid_x = 0
        centroid_y = 0
        for i in range(len(vertices) - 1):
            step = (vertices[i, 0] * vertices[i + 1, 1]) - (
                vertices[i + 1, 0] * vertices[i, 1]
            )
            area += step
            centroid_x += (vertices[i, 0] + vertices[i + 1, 0]) * step
            centroid_y += (vertices[i, 1] + vertices[i + 1, 1]) * step
        area /= 2
        # prevent division by zero - equation linked above
        if area == 0:
            area += 0.0000001
        centroid_x = (1.0 / (6.0 * area)) * centroid_x
        centroid_y = (1.0 / (6.0 * area)) * centroid_y
        # prevent centroids from escaping bounding box
        if self.constrain:
            if centroid_x < self.domains["x"]["min"]:
                centroid_x = self.domains["x"]["min"]
            if centroid_x > self.domains["x"]["max"]:
                centroid_x = self.domains["x"]["max"]
            if centroid_y < self.domains["y"]["min"]:
                centroid_y = self.domains["y"]["min"]
            if centroid_y > self.domains["y"]["max"]:
                centroid_y = self.domains["y"]["max"]
        return np.array([centroid_x, centroid_y])

    def relax(self):
        """
        Moves each point to the centroid of its cell in the voronoi
        map to "relax" the points (i.e. jitter the points so as
        to spread them out within the space).
        """
        centroids = []
        for idx in self.voronoi.point_region:
            # the region is a series of indices into self.voronoi.vertices
            # remove point at infinity, designated by index -1
            region = [i for i in self.voronoi.regions[idx] if i != -1]
            # enclose the polygon
            region = region + [region[0]]
            # get the vertices for this region
            verts = self.voronoi.vertices[region]
            # find the centroid of those vertices
            centroids.append(self.find_centroid(verts))
        self.points = np.array(centroids)
        self.constrain_points()
        self.jitter_points()
        self.build_voronoi()

    def get_points(self):
        """
        Return the input points in the new projected positions
        @returns np.array a numpy array that contains the same number
          of observations in the input points, in identical order
        """
        return self.points

```
./main.py
```python
import numpy as np
import yaml

from centerline import CenterlineGenerator
from walls import WallGenerator

CONFIG_FILE_PATH = "./conf/config.yaml"


def main():
    with open(CONFIG_FILE_PATH, "r") as f:
        config = yaml.safe_load(f)
    config = config["config"]
    config["debug_output_dir"] = "./debug"
    config["visualize"] = True

    print("=" * 60)
    print("F1Tenth Track Generator")
    print("=" * 60)

    # 1. 센터라인 생성
    print("\n[Phase 1] Generating centerline...")
    cg = CenterlineGenerator(config)
    centerline = cg.generate()

    # 센터라인 메타데이터 생성 (centerline.py 출력 형식에 맞게)
    # 실제로는 centerline.py가 (x, y, t, cumulative_distance, curvature, left_width, right_width) 반환
    # 여기서는 예시로 생성
    n_points = len(centerline)

    # 누적 거리 계산
    cumulative_distances = np.zeros(n_points)
    for i in range(1, n_points):
        dx = centerline[i, 0] - centerline[i - 1, 0]
        dy = centerline[i, 1] - centerline[i - 1, 1]
        cumulative_distances[i] = cumulative_distances[i - 1] + np.sqrt(
            dx * dx + dy * dy
        )

    # 메타데이터 배열 생성 (N x 7)
    centerline_with_metadata = np.zeros((n_points, 7))
    centerline_with_metadata[:, 0:2] = centerline  # x, y
    centerline_with_metadata[:, 2] = np.linspace(0, 1, n_points)  # t
    centerline_with_metadata[:, 3] = cumulative_distances  # cumulative_distance

    # 가상의 곡률과 너비 (실제로는 centerline.py에서 계산)
    # 사인파 형태의 곡률
    centerline_with_metadata[:, 4] = 0.1 * np.sin(np.linspace(0, 4 * np.pi, n_points))

    # 왼쪽/오른쪽 너비 (공간 경계까지의 거리 기반)
    for i in range(n_points):
        x, y = centerline[i]
        # 경계까지의 거리 (간단화)
        centerline_with_metadata[i, 5] = min(x, config["width"] - x) * 0.8  # left_width
        centerline_with_metadata[i, 6] = (
            min(y, config["height"] - y) * 0.8
        )  # right_width

    # 2. 벽 생성
    print("\n[Phase 2] Generating walls...")

    # # 커스텀 lateral offset function 정의
    # def custom_lateral_offset(**kwargs):
    #     """실제 F1Tenth 트랙 특성을 반영한 offset 함수"""
    #     # 기본값
    #     base = kwargs.get("base_width", 1.5)
    #     curvature = abs(kwargs.get("curvature", 0))
    #     is_left = kwargs.get("is_left", True)
    #     max_width = kwargs.get("left_width" if is_left else "right_width", 2.0)

    #     # 곡률 기반 조정: 곡률이 클수록 트랙을 좁힘 (안전을 위해)
    #     curvature_factor = 1.0 - min(0.5, curvature * 0.5)

    #     # 최대 너비의 70-90% 사이에서 랜덤하게 배치 (실제 설치 불규칙성 모방)
    #     random_factor = 0.7 + 0.2 * np.random.random()

    #     # 최종 offset (최대 너비 제한)
    #     offset = base * curvature_factor * random_factor
    #     return min(offset, max_width * 0.9)

    def smooth_sine_offset(**kwargs):
        """부드러운 사인파 형태의 offset 함수"""
        x = kwargs.get("x", 0)
        y = kwargs.get("y", 0)
        t = kwargs.get("t", 0)
        cumulative_distance = kwargs.get("cumulative_distance", 0)
        curvature = abs(kwargs.get("curvature", 0))
        is_left = kwargs.get("is_left", True)
        max_width = kwargs.get("left_width" if is_left else "right_width", 2.0)

        # 기본 offset (트랙 길이의 1/2)
        base = kwargs.get("base_width", 1.5)

        # 1. 곡률 기반 조정: 곡률이 클수록 약간 좁힘
        curvature_factor = 1.0 - min(0.3, curvature * 0.4)

        # 2. 주기적인 변동 (사인파) - t를 기준으로
        # 트랙 주위를 2~3번 진동하는 패턴
        periodic_factor = 1.0 + 0.15 * np.sin(t * 2 * np.pi * 2.5)

        # 3. 위치 기반 변동 (x,y 좌표에 의한)
        # 공간 내에서 자연스러운 변동을 위한 낮은 주파수 패턴
        spatial_factor = 1.0 + 0.1 * np.sin(x * 0.3) * np.cos(y * 0.3)

        # 최종 offset
        offset = base * curvature_factor * periodic_factor * spatial_factor

        # 최대 너비 제한 (안전 마진 포함)
        # return min(offset, max_width * 0.85)
        return 0.5

    # config에 lateral offset function 추가
    # config["lateral_offset_fn"] = custom_lateral_offset
    config["lateral_offset_fn"] = smooth_sine_offset

    wg = WallGenerator(config)
    walls_result = wg.generate(centerline_with_metadata)

    # 3. 결과 저장
    print("\n[Phase 3] Saving results...")

    # 센터라인 저장
    np.save("./output/centerline.npy", centerline_with_metadata)
    print(f"  - Centerline saved: {centerline_with_metadata.shape}")

    # 점유격자 저장
    if walls_result["occupancy_grid"] is not None:
        np.save("./output/wall.npy", walls_result["occupancy_grid"])
        print(f"  - Occupancy grid saved: {walls_result['occupancy_grid'].shape}")

    # 벽 점들 저장
    if walls_result["wall_points"] is not None:
        np.save("./output/wall_points.npy", walls_result["wall_points"])
        print(f"  - Wall points saved: {len(walls_result['wall_points'])} points")

    # 메타데이터 저장
    import json

    with open("./output/metadata.json", "w") as f:
        json.dump(walls_result["metadata"], f, indent=2)
    print("  - Metadata saved")

    print("\n" + "=" * 60)
    print("GENERATION COMPLETE!")
    print("=" * 60)


if __name__ == "__main__":
    main()

```
./oldduct.py
```python
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

```
./pyproject.toml
```
[project]
name = "f1tenth-racetrack-generator"
version = "0.1.0"
description = "Add your description here"
readme = "README.md"
requires-python = ">=3.13"
dependencies = [
    "concave-hull>=0.1.2",
    "matplotlib>=3.10.8",
    "numpy>=2.4.2",
    "pyyaml>=6.0.3",
    "scipy>=1.17.0",
    "shapely>=2.1.2",
]

```
./track_width_profile_functions.py
```python
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

```
./walls.py
```python
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

```
