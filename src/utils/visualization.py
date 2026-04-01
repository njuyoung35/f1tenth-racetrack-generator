"""
Visualization utilities for plotting tracks, centerlines, and metrics.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
from matplotlib.collections import LineCollection
from typing import Optional, Union, List, Dict, Any
import os
from pathlib import Path

# Import data structures (relative imports)
from ..core.dataclass import CenterlineData, TrackData
from ..air_duct.base import AirDuctsData, OrientedBoundingBox


def setup_plot_style():
    """Set a consistent plot style."""
    plt.style.use('seaborn-v0_8-darkgrid')
    plt.rcParams['figure.figsize'] = (10, 8)
    plt.rcParams['font.size'] = 12


def plot_track(track: TrackData,
               air_ducts: Optional[AirDuctsData] = None,
               save_path: Optional[Union[str, Path]] = None,
               show: bool = True,
               title: str = "Generated Track") -> None:
    """
    Plot the track with centerline, boundaries, and optionally air ducts.

    Args:
        track: TrackData object
        air_ducts: Optional AirDuctsData object
        save_path: If provided, save the figure to this path
        show: Whether to display the plot
        title: Title of the plot
    """
    setup_plot_style()
    fig, ax = plt.subplots()

    # Plot centerline
    centerline = track.centerline.points
    ax.plot(centerline[:, 0], centerline[:, 1], 'b-', linewidth=2, label='Centerline')

    # Plot boundaries
    left = track.left_boundary
    right = track.right_boundary
    ax.plot(left[:, 0], left[:, 1], 'r-', linewidth=1.5, alpha=0.7, label='Left boundary')
    ax.plot(right[:, 0], right[:, 1], 'g-', linewidth=1.5, alpha=0.7, label='Right boundary')

    # Fill the track interior (optional)
    # Concatenate left and reversed right to form a polygon
    track_polygon = np.vstack([left, right[::-1]])
    ax.fill(track_polygon[:, 0], track_polygon[:, 1], alpha=0.1, color='gray')

    # Plot air ducts as rectangles (or OBBs)
    if air_ducts is not None:
        for obb in air_ducts.obbs:
            # OBB corners
            corners = obb.corners
            poly = Polygon(corners, closed=True, alpha=0.6, facecolor='orange', edgecolor='black', linewidth=1)
            ax.add_patch(poly)

    # Set equal aspect ratio
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(title)
    ax.legend()

    # Adjust limits with margin
    all_points = np.vstack([left, right, centerline])
    if air_ducts:
        for obb in air_ducts.obbs:
            all_points = np.vstack([all_points, obb.corners])
    margin = 0.1 * (all_points.max(axis=0) - all_points.min(axis=0))
    ax.set_xlim(all_points[:, 0].min() - margin[0], all_points[:, 0].max() + margin[0])
    ax.set_ylim(all_points[:, 1].min() - margin[1], all_points[:, 1].max() + margin[1])

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_centerline(centerline: CenterlineData,
                    save_path: Optional[Union[str, Path]] = None,
                    show: bool = True,
                    colormap: str = 'viridis') -> None:
    """
    Plot centerline with curvature as color.

    Args:
        centerline: CenterlineData object
        save_path: If provided, save the figure
        show: Whether to display
        colormap: Matplotlib colormap for curvature
    """
    setup_plot_style()
    fig, ax = plt.subplots()

    points = centerline.points
    curvature = centerline.curvature

    # Create line segments with color based on curvature
    segments = np.stack([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(vmin=np.min(np.abs(curvature)), vmax=np.max(np.abs(curvature)))
    lc = LineCollection(segments, cmap=colormap, norm=norm, linewidth=2)
    lc.set_array(np.abs(curvature[1:]))  # color by absolute curvature
    ax.add_collection(lc)

    # Add colorbar
    cbar = plt.colorbar(lc, ax=ax)
    cbar.set_label('Curvature [1/m]')

    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Centerline with Curvature')

    # Add start point marker
    ax.plot(points[0, 0], points[0, 1], 'go', markersize=8, label='Start')
    ax.plot(points[-1, 0], points[-1, 1], 'ro', markersize=8, label='End')
    ax.legend()

    # Limits
    margin = 0.1 * (points.max(axis=0) - points.min(axis=0))
    ax.set_xlim(points[:, 0].min() - margin[0], points[:, 0].max() + margin[0])
    ax.set_ylim(points[:, 1].min() - margin[1], points[:, 1].max() + margin[1])

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_width_profile(track: TrackData,
                       save_path: Optional[Union[str, Path]] = None,
                       show: bool = True) -> None:
    """
    Plot track width as a function of arc length.

    Args:
        track: TrackData object
        save_path: Save path
        show: Whether to display
    """
    setup_plot_style()
    fig, ax = plt.subplots()

    s = track.centerline.s
    width = track.width_profile

    ax.plot(s, width, 'b-', linewidth=2)
    ax.fill_between(s, 0, width, alpha=0.3, color='blue')
    ax.set_xlabel('Arc length [m]')
    ax.set_ylabel('Track width [m]')
    ax.set_title('Track Width Profile')
    ax.grid(True, alpha=0.5)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_curvature_profile(centerline: CenterlineData,
                           save_path: Optional[Union[str, Path]] = None,
                           show: bool = True) -> None:
    """
    Plot curvature as a function of arc length.

    Args:
        centerline: CenterlineData object
        save_path: Save path
        show: Whether to display
    """
    setup_plot_style()
    fig, ax = plt.subplots()

    s = centerline.s
    curvature = centerline.curvature

    ax.plot(s, curvature, 'r-', linewidth=2)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax.set_xlabel('Arc length [m]')
    ax.set_ylabel('Curvature [1/m]')
    ax.set_title('Curvature Profile')
    ax.grid(True, alpha=0.5)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_metrics_comparison(metrics_dict: Dict[str, Dict[str, float]],
                            save_path: Optional[Union[str, Path]] = None,
                            show: bool = True) -> None:
    """
    Create a bar chart comparing multiple metrics across different runs.

    Args:
        metrics_dict: Dictionary where keys are run names and values are metric dicts
        save_path: Save path
        show: Whether to display
    """
    setup_plot_style()
    fig, ax = plt.subplots(figsize=(12, 6))

    # Extract metric names (assume all runs have same metric keys)
    metric_names = list(next(iter(metrics_dict.values())).keys())
    run_names = list(metrics_dict.keys())

    # Prepare data for bar groups
    x = np.arange(len(metric_names))
    width = 0.8 / len(run_names)  # bar width per run
    colors = plt.cm.tab10(np.linspace(0, 1, len(run_names)))

    for i, (run_name, metrics) in enumerate(metrics_dict.items()):
        values = [metrics[name] for name in metric_names]
        offset = (i - len(run_names)/2 + 0.5) * width
        ax.bar(x + offset, values, width, label=run_name, color=colors[i])

    ax.set_xticks(x)
    ax.set_xticklabels(metric_names, rotation=45, ha='right')
    ax.set_ylabel('Metric Value')
    ax.set_title('Metrics Comparison Across Runs')
    ax.legend()
    ax.grid(True, axis='y', alpha=0.5)

    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_tortuosity_vs_entropy(metrics_list: List[Dict[str, float]],
                               run_names: List[str],
                               save_path: Optional[Union[str, Path]] = None,
                               show: bool = True) -> None:
    """
    Scatter plot of tortuosity vs curvature entropy for multiple runs.

    Args:
        metrics_list: List of metric dictionaries
        run_names: Corresponding run names
        save_path: Save path
        show: Whether to display
    """
    setup_plot_style()
    fig, ax = plt.subplots()

    tortuosities = []
    entropies = []
    for m in metrics_list:
        tortuosities.append(m.get('tortuosity', np.nan))
        entropies.append(m.get('curvature_entropy', np.nan))

    # Filter out NaN
    valid = [i for i in range(len(tortuosities)) if not np.isnan(tortuosities[i]) and not np.isnan(entropies[i])]
    tortuosities = [tortuosities[i] for i in valid]
    entropies = [entropies[i] for i in valid]
    names = [run_names[i] for i in valid]

    ax.scatter(tortuosities, entropies, s=100, c='blue', alpha=0.7)
    for name, t, e in zip(names, tortuosities, entropies):
        ax.annotate(name, (t, e), xytext=(5, 5), textcoords='offset points', fontsize=8)

    ax.set_xlabel('Tortuosity')
    ax.set_ylabel('Curvature Entropy')
    ax.set_title('Tortuosity vs Curvature Entropy')
    ax.grid(True, alpha=0.5)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
    if show:
        plt.show()
    plt.close()


def plot_all_visualizations(track: TrackData,
                            air_ducts: Optional[AirDuctsData] = None,
                            output_dir: Union[str, Path] = None,
                            show: bool = False) -> None:
    """
    Generate all standard visualizations and save them to output_dir.

    Args:
        track: TrackData
        air_ducts: Optional AirDuctsData
        output_dir: Directory to save plots
        show: Whether to display (default False)
    """
    if output_dir is None:
        output_dir = Path.cwd() / "plots"
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_track(track, air_ducts, save_path=output_dir / "track.png", show=show)
    plot_centerline(track.centerline, save_path=output_dir / "centerline_curvature.png", show=show)
    plot_width_profile(track, save_path=output_dir / "width_profile.png", show=show)
    plot_curvature_profile(track.centerline, save_path=output_dir / "curvature_profile.png", show=show)