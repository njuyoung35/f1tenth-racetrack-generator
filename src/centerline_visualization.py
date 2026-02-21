"""
Visualization utilities for centerline generation.
Separated from core logic for better modularity.
"""

import os

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi


def visualize_initial_points(points, width, height, save_path):
    """
    Visualize initial random points within boundary.

    Args:
        points: (n, 2) array of points
        width: space width
        height: space height
        save_path: full path to save the figure
    """
    plt.figure(figsize=(12, 8))
    plt.scatter(points[:, 0], points[:, 1], s=10, c="blue", alpha=0.6)
    plt.plot(
        [0, width, width, 0, 0],
        [0, 0, height, height, 0],
        "r-",
        linewidth=2,
    )
    plt.title("Initial Random Points")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")


def visualize_voronoi(
    vor, title, save_path, width=None, height=None, points_only=False
):
    """
    Visualize Voronoi diagram.

    Args:
        vor: Voronoi diagram
        title: plot title
        save_path: full path to save the figure
        width: optional, if provided filters vertices within bounds
        height: optional, if provided filters vertices within bounds
        points_only: if True, only show points (no edges/vertices)
    """
    plt.figure(figsize=(12, 8))

    if not points_only:
        # Draw Voronoi edges
        for simplex in vor.ridge_vertices:
            if -1 not in simplex:
                p1, p2 = vor.vertices[simplex[0]], vor.vertices[simplex[1]]
                plt.plot(
                    [p1[0], p2[0]], [p1[1], p2[1]], "orange", linewidth=1, alpha=0.6
                )

        # Draw vertices
        if width is not None and height is not None:
            # Filter vertices within bounds
            valid_vertices = []
            for vertex in vor.vertices:
                if 0 <= vertex[0] <= width and 0 <= vertex[1] <= height:
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
        else:
            plt.scatter(
                vor.vertices[:, 0],
                vor.vertices[:, 1],
                s=10,
                c="orange",
                alpha=0.5,
                label="Voronoi Vertices",
            )

    # Draw seed points
    plt.scatter(
        vor.points[:, 0],
        vor.points[:, 1],
        s=20,
        c="blue",
        alpha=0.8,
        label="Seed Points",
    )

    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.legend()
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")


def visualize_grid_selection(
    vor,
    selected_indices,
    grid_cols,
    grid_rows,
    virtual_grid_width,
    width,
    height,
    save_path,
):
    """
    Visualize virtual grid and selected regions.

    Args:
        vor: Voronoi diagram
        selected_indices: list of selected region indices
        grid_cols: number of grid columns
        grid_rows: number of grid rows
        virtual_grid_width: size of each grid cell
        width: space width
        height: space height
        save_path: full path to save the figure
    """
    plt.figure(figsize=(12, 8))

    # Draw boundary
    plt.plot(
        [0, width, width, 0, 0],
        [0, 0, height, height, 0],
        "k-",
        linewidth=2,
        label="Boundary",
    )

    # Draw Voronoi edges (within bounds)
    for simplex in vor.ridge_vertices:
        if -1 not in simplex:
            p1, p2 = vor.vertices[simplex[0]], vor.vertices[simplex[1]]
            if (
                0 <= p1[0] <= width
                and 0 <= p1[1] <= height
                and 0 <= p2[0] <= width
                and 0 <= p2[1] <= height
            ):
                plt.plot(
                    [p1[0], p2[0]], [p1[1], p2[1]], "orange", linewidth=1, alpha=0.3
                )

    # Draw Voronoi vertices (within bounds)
    valid_vertices = []
    for vertex in vor.vertices:
        if 0 <= vertex[0] <= width and 0 <= vertex[1] <= height:
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

    # Draw virtual grid
    for i in range(grid_cols + 1):
        x = i * virtual_grid_width
        plt.axvline(x=x, color="gray", linestyle="--", alpha=0.5)

    for j in range(grid_rows + 1):
        y = j * virtual_grid_width
        plt.axhline(y=y, color="gray", linestyle="--", alpha=0.5)

    # Draw selected regions
    if len(selected_indices) > 0:
        selected_points = vor.points[selected_indices]
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
    plt.xlim(-1, width + 1)
    plt.ylim(-1, height + 1)
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")


def visualize_concave_hull(centroids, hull_points, save_path):
    """
    Visualize concave hull.

    Args:
        centroids: array of centroid points
        hull_points: array of hull points (closed polygon)
        save_path: full path to save the figure
    """
    plt.figure(figsize=(12, 8))

    # Draw centroids
    plt.scatter(
        centroids[:, 0],
        centroids[:, 1],
        s=50,
        c="blue",
        alpha=0.6,
        label="Centroids",
    )

    # Draw concave hull
    plt.plot(
        hull_points[:, 0],
        hull_points[:, 1],
        "r-",
        linewidth=2,
        label="Concave Hull",
    )
    plt.scatter(
        hull_points[:, 0],
        hull_points[:, 1],
        s=30,
        c="red",
        marker="o",
    )

    plt.title("Concave Hull from Selected Centroids")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")


def visualize_curvature_check(
    original_points, centerline, curvature, iteration, curvature_threshold, save_path
):
    """
    Visualize curvature check results.

    Args:
        original_points: original points before interpolation
        centerline: interpolated centerline points
        curvature: array of curvature values
        iteration: current iteration number
        curvature_threshold: threshold value for highlighting
        save_path: full path to save the figure
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))

    # Left: points and centerline
    ax1.scatter(
        original_points[:, 0],
        original_points[:, 1],
        s=20,
        c="blue",
        alpha=0.6,
        label="Original Points",
    )
    ax1.plot(
        centerline[:, 0],
        centerline[:, 1],
        "r-",
        linewidth=2,
        label="Centerline",
    )
    ax1.set_title(f"Centerline - Iteration {iteration}")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axis("equal")

    # Right: curvature distribution
    ax2.plot(curvature, "b-", linewidth=1)
    ax2.axhline(
        y=curvature_threshold,
        color="r",
        linestyle="--",
        alpha=0.7,
        label="Curvature Threshold",
    )
    ax2.axhline(
        y=-curvature_threshold,
        color="r",
        linestyle="--",
        alpha=0.7,
    )
    ax2.fill_between(
        range(len(curvature)),
        -curvature_threshold,
        curvature_threshold,
        alpha=0.1,
        color="green",
    )
    ax2.set_title("Curvature Distribution")
    ax2.set_xlabel("Point Index")
    ax2.set_ylabel("Curvature")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")


def visualize_final_centerline(original_points, centerline, save_path):
    """
    Visualize final centerline.

    Args:
        original_points: original points used for generation
        centerline: final centerline points
        save_path: full path to save the figure
    """
    plt.figure(figsize=(12, 8))

    # Original points
    plt.scatter(
        original_points[:, 0],
        original_points[:, 1],
        s=20,
        c="blue",
        alpha=0.3,
        label="Original Points",
    )

    # Final centerline
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
    plt.savefig(save_path)
    plt.close()
    print(f"  - Saved: {save_path}")
