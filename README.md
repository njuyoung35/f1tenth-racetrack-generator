# f1tenth-racetrack-generator

## file structure

```
f1tenth_track_gen/
├── config/                                 # Configuration files (YAML)
│   ├── experiment.yaml                     # Main experiment: list of runs, global settings
│   ├── centerline/                         # Component-specific configs (optional)
│   │   ├── voronoi.yaml
│   │   ├── cmaes.yaml
│   │   └── ...
│   ├── track/
│   │   ├── curvature_width.yaml
│   │   ├── stochastic.yaml
│   │   └── ...
│   └── air_duct/
│       ├── obb_chain.yaml
│       └── probabilistic.yaml
├── src/                                    # Source code
│   ├── core/                               # Core data structures and interfaces
│   │   ├── interfaces.py                   # Abstract base classes for components
│   │   ├── track_data.py                   # Data classes (Centerline, Track, AirDucts)
│   │   └── metrics.py                      # Functions to compute track metrics
│   ├── centerline/                         # Centerline generation algorithms
│   │   ├── base.py                         # Base class
│   │   ├── procedural_voronoi.py           # Voronoi cell-based
│   │   ├── optimization_cmaes.py           # CMA-ES optimization
│   │   ├── preference_learning.py          # Procedural + filtering (preference learning)
│   │   └── ...
│   ├── track/                              # Track generation (width profile)
│   │   ├── base.py
│   │   ├── curvature_width.py              # Curvature-coupled width
│   │   ├── stochastic_width.py             # Perlin noise based
│   │   └── combined.py                     # Combines both
│   ├── air_duct/                           # Air duct placement algorithms
│   │   ├── base.py
│   │   ├── obb_chain.py                    # OBB chain with greedy fitting & optimization
│   │   ├── probabilistic.py                # Probabilistic distribution model
│   │   └── ...
│   ├── utils/                              # Helper modules
│   │   ├── geometry.py                     # Spline interpolation, offset, curvature, etc.
│   │   ├── io.py                           # Save/load track to YAML, occupancy grid, USD
│   │   ├── visualization.py                # Plot centerline, track, metrics
│   │   └── logging.py                      # Experiment logger (e.g., using MLflow, tensorboard)
│   └── main.py                             # Main entry point (runs experiments)
├── experiments/                            # Experiment outputs
│   ├── logs/                               # Per-run logs (metrics, config, generated files)
│   └── results/                            # Aggregated statistics, plots, comparison tables
├── tests/                                  # Unit tests for components
├── requirements.txt                        # Python dependencies
└── README.md                               # Documentation
```
