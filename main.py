import yaml
import importlib
import os
from src.core.dataclass import CenterlineData, TrackData, AirDuctsData
from src.core.metrics import compute_all_metrics
from src.utils.io import save_track_to_yaml, save_occupancy_grid, save_to_usd
from src.utils.visualization import plot_track

def load_component(component_type, config, seed):
    # Import module dynamically
    if component_type == "procedural_voronoi":
        from src.centerline.procedural_voronoi import VoronoiCenterlineGenerator
        return VoronoiCenterlineGenerator(config, seed)
    elif component_type == "curvature_coupled":
        from src.track.curvature_coupled import CurvatureCoupledTrackGenerator
        return CurvatureCoupledTrackGenerator(config, seed)
    elif component_type == "obb_chain":
        from src.air_duct.obb_chain import OBBChainAirDuctPlacer
        return OBBChainAirDuctPlacer(config, seed)
    # elif component_type == "optimization_cmaes":
    #     from src.centerline.optimization_cmaes import CMAESCenterlineGenerator
    #     return CMAESCenterlineGenerator(config, seed)
    # ... etc.
    else:
        raise ValueError(f"Unknown component type: {component_type}")

def main():
    with open("config/experiment.yaml", "r") as f:
        exp_config = yaml.safe_load(f)

    global_config = exp_config.get("global", {})
    base_seed = global_config.get("seed", None)
    output_dir = global_config.get("output_dir", "experiments/")

    for run in exp_config["runs"]:
        run_name = run["name"]
        run_dir = os.path.join(output_dir, "logs", run_name)
        os.makedirs(run_dir, exist_ok=True)

        # Get seed for this run (override if specified)
        seed = run.get("seed", base_seed)

        # Instantiate centerline generator
        cl_config = run["centerline"]["params"]
        cl_generator = load_component(run["centerline"]["type"], cl_config, seed)
        centerline = cl_generator.generate()
        
        from src.utils.visualization import plot_centerline
        plot_centerline(centerline, save_path=os.path.join(run_dir, "centerline.png"))
        
        return

        # Instantiate track generator
        tr_config = run["track"]["params"]
        tr_generator = load_component(run["track"]["type"], tr_config, seed)
        track = tr_generator.generate(centerline)

        # Instantiate air duct placer
        ad_config = run["air_duct"]["params"]
        ad_placer = load_component(run["air_duct"]["type"], ad_config, seed)
        air_ducts = ad_placer.place(track)

        # Compute metrics
        metrics = compute_all_metrics(exp_config, centerline, track, air_ducts)
        
        # Log metrics
        print(f"Track metrics: {metrics['total_length']:.2f}m length, "
              f"technical_score: {metrics['technical_score']:.1f}, "
              f"overtaking_opportunities: {metrics['overtaking_opportunities']}")

        # Save metrics
        import json
        with open(os.path.join(run_dir, 'metrics.json'), 'w') as f:
            json.dump(metrics, f, indent=2)
    
        # Save outputs
        save_track_to_yaml(track, air_ducts, metrics, run_dir)
        save_occupancy_grid(track, air_ducts, run_dir)   # .png
        # Optionally save to USD if needed
        save_to_usd(track, air_ducts, run_dir)

        # Visualize (optional)
        if global_config.get("visualize", False):
            plot_track(track, air_ducts, save_path=os.path.join(run_dir, "track.png"))

        # Log metrics (e.g., to a CSV or MLflow)
        # ...

    # After all runs, aggregate results and generate comparison
    # ...

if __name__ == "__main__":
    main()