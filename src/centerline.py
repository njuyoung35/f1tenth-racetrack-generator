import os

import numpy as np
from concave_hull import concave_hull
from lloyd import Field
from scipy import interpolate
from scipy.spatial import Voronoi
from shapely.geometry import Polygon

from .centerline_visualization import (
    visualize_concave_hull,
    visualize_curvature_check,
    visualize_final_centerline,
    visualize_grid_selection,
    visualize_initial_points,
    visualize_voronoi,
)

# Constants
RANDOM_POINT_MARGIN = 0.25
CONCAVITY = 0.33


class CenterlineGenerator:
    """
    Generates a smooth closed-loop centerline for a racetrack using
    Voronoi diagrams, Lloyd's algorithm, and concave hulls.
    """

    def __init__(self, config):
        """
        Initialize the centerline generator with configuration.

        Args:
            config: Dictionary containing all configuration parameters
                   (should include 'space', 'centerline', 'output' sections)
        """
        # Extract configuration sections
        space_config = config.get("space", {})
        centerline_config = config.get("centerline", {})
        output_config = config.get("output", {})

        # Space parameters
        self.width = space_config.get("width", 20.0)
        self.height = space_config.get("height", 10.0)

        # Centerline generation parameters
        self.n_points = centerline_config.get("n_points", 60)
        self.max_lloyd_iterations = centerline_config.get("max_lloyd_iterations", 10)
        self.max_verifying_iterations = centerline_config.get(
            "max_verifying_iterations", 50
        )
        self.virtual_grid_width = centerline_config.get("virtual_grid_width", 5.0)
        self.virtual_grid_coverage = centerline_config.get("virtual_grid_coverage", 0.8)
        self.curvature_threshold = centerline_config.get("curvature_threshold", 0.3)

        # Output/debug parameters
        self.visualize = output_config.get("visualize", False)
        self.debug_output_dir = output_config.get("debug_output_dir", "debug")
        self.random_seed = output_config.get("random_seed", None)

        # Set random seed if provided
        if self.random_seed is not None:
            np.random.seed(self.random_seed)

        # Create debug directory if needed
        if self.visualize:
            os.makedirs(self.debug_output_dir, exist_ok=True)

    def generate_points(self) -> np.ndarray:
        """
        Generate initial random points within the space boundary.

        Returns:
            np.ndarray: (n_points, 2) array of point coordinates
        """
        margin = RANDOM_POINT_MARGIN
        points = np.random.rand(self.n_points, 2)
        points[:, 0] = points[:, 0] * (self.width - 2 * margin) + margin
        points[:, 1] = points[:, 1] * (self.height - 2 * margin) + margin

        if self.visualize:
            save_path = os.path.join(
                self.debug_output_dir, "00_initial_random_points.png"
            )
            visualize_initial_points(points, self.width, self.height, save_path)

        return points

    def run_lloyds_algorithm(self, points) -> Voronoi:
        """
        Apply Lloyd's algorithm to relax point distribution.

        Args:
            points: Initial point array

        Returns:
            Voronoi: Final Voronoi diagram after relaxation
        """
        field = Field(points)

        if self.visualize:
            save_path = os.path.join(self.debug_output_dir, "01_initial_voronoi.png")
            visualize_voronoi(
                field.voronoi,
                "Initial Voronoi Diagram",
                save_path,
                self.width,
                self.height,
            )

        for iteration in range(self.max_lloyd_iterations):
            field.relax()
            if self.visualize:
                save_path = os.path.join(
                    self.debug_output_dir, f"02_lloyd_iteration_{iteration + 1:02d}.png"
                )
                visualize_voronoi(
                    field.voronoi,
                    f"Lloyd Iteration {iteration + 1}",
                    save_path,
                    self.width,
                    self.height,
                )

        return field.voronoi

    def select_regions_from_virtual_grid(self, vor: Voronoi) -> list:
        """
        Select Voronoi regions using a virtual grid sampling strategy.

        Args:
            vor: Voronoi diagram

        Returns:
            list: Indices of selected regions
        """
        grid_cols = int(np.ceil(self.width / self.virtual_grid_width))
        grid_rows = int(np.ceil(self.height / self.virtual_grid_width))
        k_options = [3, 4, 5]  # Randomly choose k for diversity

        selected_regions = []

        for i in range(grid_cols):
            for j in range(grid_rows):
                if np.random.random() > self.virtual_grid_coverage:
                    continue

                cell_center_x = (
                    i * self.virtual_grid_width + self.virtual_grid_width / 2
                )
                cell_center_y = (
                    j * self.virtual_grid_width + self.virtual_grid_width / 2
                )

                if cell_center_x > self.width or cell_center_y > self.height:
                    continue

                k = np.random.choice(k_options)
                distances = np.linalg.norm(
                    vor.points - np.array([cell_center_x, cell_center_y]), axis=1
                )
                top_k_indices = np.argsort(distances)[:k]

                # Weight by inverse distance
                top_k_distances = distances[top_k_indices]
                weights = 1.0 / (top_k_distances + 0.1)
                weights = weights / weights.sum()

                chosen_idx = np.random.choice(top_k_indices, p=weights)
                selected_regions.append(chosen_idx)

        # Remove duplicates and ensure minimum count
        selected_regions = list(set(selected_regions))
        if len(selected_regions) < 3:
            all_regions = list(range(len(vor.points)))
            additional_needed = 3 - len(selected_regions)
            available = list(set(all_regions) - set(selected_regions))
            if available:
                additional = np.random.choice(
                    available,
                    size=min(additional_needed, len(available)),
                    replace=False,
                )
                selected_regions.extend(additional)

        print(
            f"Selected {len(selected_regions)} regions from {grid_cols * grid_rows} cells"
        )

        if self.visualize:
            save_path = os.path.join(self.debug_output_dir, "03_grid_selection.png")
            visualize_grid_selection(
                vor,
                selected_regions,
                grid_cols,
                grid_rows,
                self.virtual_grid_width,
                self.width,
                self.height,
                save_path,
            )

        return selected_regions

    def create_concave_hull_polygon(
        self, vor: Voronoi, selected_region_indices: list
    ) -> np.ndarray:
        """
        Create a concave hull polygon from selected region seed points.

        Args:
            vor: Voronoi diagram
            selected_region_indices: Indices of selected regions

        Returns:
            np.ndarray: Closed polygon points
        """
        selected_points = [vor.points[idx] for idx in selected_region_indices]

        if len(selected_points) < 3:
            raise ValueError(
                f"Too few points ({len(selected_points)}) for concave hull"
            )

        points_array = np.array(selected_points)
        hull_points = concave_hull(
            points_array,
            concavity=CONCAVITY,
            length_threshold=0.0,
        )

        # Close the polygon
        if len(hull_points) > 0:
            if not np.array_equal(hull_points[0], hull_points[-1]):
                hull_points = np.vstack([hull_points, hull_points[0]])

        if self.visualize:
            save_path = os.path.join(self.debug_output_dir, "04_concave_hull.png")
            visualize_concave_hull(points_array, hull_points, save_path)

        return hull_points

    def interpolate_polygon_with_voronoi_vertices(
        self, vor: Voronoi, polygon: np.ndarray
    ) -> np.ndarray:
        """
        Interpolate polygon using nearby Voronoi vertices and refine by curvature.

        Args:
            vor: Voronoi diagram
            polygon: Input polygon points

        Returns:
            np.ndarray: Refined points
        """
        interpolated_points = []

        for i in range(len(polygon) - 1):
            start = polygon[i]
            end = polygon[i + 1]

            segment_length = np.linalg.norm(end - start)
            max_distance = segment_length * 0.3

            # Find nearby Voronoi vertices
            nearby = []
            for vertex in vor.vertices:
                distance = self._point_to_line_distance(vertex, start, end)
                if distance < max_distance and self._is_point_between(
                    vertex, start, end, margin=1.0
                ):
                    nearby.append(vertex)

            # Limit vertices per segment
            max_vertices = max(3, int(segment_length / self.virtual_grid_width * 2))
            if len(nearby) > max_vertices:
                distances = [
                    self._point_to_line_distance(v, start, end) for v in nearby
                ]
                sorted_indices = np.argsort(distances)
                nearby = [nearby[i] for i in sorted_indices[:max_vertices]]

            interpolated_points.append(start)
            if nearby:
                sorted_vertices = sorted(
                    nearby, key=lambda v: np.linalg.norm(v - start)
                )
                interpolated_points.extend(sorted_vertices)

        interpolated_points.append(polygon[-1])
        return self._refine_with_curvature_check(np.array(interpolated_points))

    def _refine_with_curvature_check(self, points: np.ndarray) -> np.ndarray:
        """
        Iteratively refine points to satisfy curvature constraint.

        Args:
            points: Initial point array

        Returns:
            np.ndarray: Points satisfying curvature constraint
        """
        max_iterations = 10

        for iteration in range(max_iterations):
            try:
                tck, u = interpolate.splprep(
                    [points[:, 0], points[:, 1]],
                    s=0,
                    per=True,
                )

                u_fine = np.linspace(0, 1, len(points) * 10)
                x_fine, y_fine = interpolate.splev(u_fine, tck)
                curvature = self._calculate_spline_curvature(tck, u_fine)

                if np.max(np.abs(curvature)) <= self.curvature_threshold:
                    centerline = np.column_stack([x_fine, y_fine])

                    if self.visualize:
                        save_path = os.path.join(
                            self.debug_output_dir,
                            f"05_curvature_check_{iteration:02d}.png",
                        )
                        visualize_curvature_check(
                            points,
                            centerline,
                            curvature,
                            iteration,
                            self.curvature_threshold,
                            save_path,
                        )

                    return centerline
                else:
                    # Remove point with highest curvature
                    high_idx = np.where(np.abs(curvature) > self.curvature_threshold)[0]
                    if len(high_idx) > 0:
                        peak_idx = high_idx[np.argmax(np.abs(curvature[high_idx]))]
                        peak_param = u_fine[peak_idx]
                        remove_idx = np.argmin(np.abs(u - peak_param))

                        if len(points) > 10:
                            points = np.delete(points, remove_idx, axis=0)
                        else:
                            break
            except Exception as e:
                print(f"Curvature refinement iteration {iteration} failed: {e}")
                break

        return points

    def _final_interpolation(self, points: np.ndarray) -> np.ndarray:
        """
        Final spline interpolation to create smooth centerline.

        Args:
            points: Input points

        Returns:
            np.ndarray: Smooth centerline points
        """
        try:
            tck, u = interpolate.splprep([points[:, 0], points[:, 1]], s=0, per=True)

            track_length = self._calculate_track_length(points)
            n_interp_points = max(100, int(track_length * 20))  # Finer resolution

            u_fine = np.linspace(0, 1, n_interp_points)
            x_fine, y_fine = interpolate.splev(u_fine, tck)

            centerline = np.column_stack([x_fine, y_fine])

            if self.visualize:
                save_path = os.path.join(
                    self.debug_output_dir, "06_final_centerline.png"
                )
                visualize_final_centerline(points, centerline, save_path)

            return centerline
        except Exception as e:
            print(f"Final interpolation failed: {e}")
            return points

    def generate(self) -> np.ndarray:
        """
        Main method to generate centerline.

        Returns:
            np.ndarray: Generated centerline points (N x 2)
        """
        print("\n" + "=" * 60)
        print("CENTERLINE GENERATION")
        print("=" * 60)

        for attempt in range(self.max_verifying_iterations):
            print(f"\nAttempt {attempt + 1}/{self.max_verifying_iterations}")

            points = self.generate_points()
            vor = self.run_lloyds_algorithm(points)
            selected_regions = self.select_regions_from_virtual_grid(vor)
            concave_polygon = self.create_concave_hull_polygon(vor, selected_regions)
            centerline = self.interpolate_polygon_with_voronoi_vertices(
                vor, concave_polygon
            )
            centerline = self._final_interpolation(centerline)

            # Validate
            if Polygon(centerline).is_valid:
                print("\n✓ Valid centerline generated!")
                return centerline

        raise RuntimeError(
            f"Failed to generate valid centerline within {self.max_verifying_iterations} attempts"
        )

    # Utility methods (kept in main class)
    def _calculate_spline_curvature(self, tck, u_values):
        """Calculate curvature of a spline at given parameter values."""
        dx, dy = interpolate.splev(u_values, tck, der=1)
        ddx, ddy = interpolate.splev(u_values, tck, der=2)
        curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2) ** 1.5
        return np.nan_to_num(curvature, nan=0.0, posinf=0.0, neginf=0.0)

    def _point_to_line_distance(self, point, line_start, line_end):
        """Calculate distance from point to line segment."""
        line_vec = line_end - line_start
        point_vec = point - line_start
        line_length = np.dot(line_vec, line_vec)

        if line_length == 0:
            return np.linalg.norm(point - line_start)

        t = max(0, min(1, np.dot(point_vec, line_vec) / line_length))
        projection = line_start + t * line_vec
        return np.linalg.norm(point - projection)

    def _is_point_between(self, point, line_start, line_end, margin=0.5):
        """Check if point lies on or near the line segment."""
        line_vec = line_end - line_start
        point_vec = point - line_start
        t = np.dot(point_vec, line_vec) / np.dot(line_vec, line_vec)
        return -margin <= t <= (1 + margin)

    def _calculate_track_length(self, path: np.ndarray) -> float:
        """Calculate total length of a path."""
        return np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
