# SPEC.md

- this algorithm generates f1tenth racetrack randomly with abstract geometric/functional/meta features.
- it not merely only use procedural generation, but also apply optimization methods in algorithm.
- this algorithm is designed to be integrated into RL training environment like issac sim.

## Algorithm Step

1. centerline generation
2. track(wall) generation
3. air duct placement
4. result outputs

## algorithm implementation candidates

1. centerline generation
  - procedural content generation
        - voronoi cell-based method (https://github.com/mvanlobensels/random-track-generator)
  - optimization method (with objective function)
    - gradient-based
    - sampling-based
      - [X] CMA-ES - for optimization non-linaer objective function
    - what about update logic?
      - via: RLHF rewarding modeling
        - algorithm suggest 2 candidate centerline for each steps, human select better one (pairwise comparison)
      - [X] via: procedural generation + filtering (preference learning)
        - initialize points with naive procedural algorithm
        - evaluate with objective function
        - update with sampling: perturbating control points, and find gradient to increase objective function, and update samples.
  - candidates of representation of centerline:
    - [X] spline (B-Spline or Catmull-Rom)
    - Frenet Frame-based
    - raw Cartesian-coordinates points
2. track(wall) generation
  - given centerline points from previous step, in this step we calculate track(wall).
  - i'm considering 'distance profile function' `w(s)`. (of course it can include more parameters like curvature, x, y, more info).
  - some candidates to design `w(s)`:
    - curvautre-coupled width: `w(s) = w_base + alpha / (1 + beta * |kappa(s)|)`
      - automatically widen the track where curvature is low (straights) and narrow it at apexes.
    - stochastic perturbation: apply a perlin noise layer to `w(s)` to create organic irregularities
3. air duct placement
  - Oriented Bounding Boxes (OBB): modeling wall as list of OBBs ?
  - the chain algorithm
    1. offseting: Calculate the left and right boundary curves by offsetting the centerline by ±w(s)/2.
    2. greedy fitting: Starting at s=0, place the first duct along the tangent of the boundary curve.
    3. Constraint Solving: Since ducts are rigid, the "end" of one duct must touch the "start" of the next. This creates a "chord" across the curve.
      - If the curvature is too high, the "gap" on the outer edge of the duct joint becomes too large.
      - Optimization: Adjust the duct angles to minimize the Hausdorff distance between the discrete duct chain and the ideal smooth boundary curve.
  - probalistic distribution model
  - some geometrical-statistics-optimization fusioned idea..

* [X] mark indicates what i choose.

## Data Flow

1. centerline generation
  - input
    - required: width, height, 
    - optional: sampling point count, max_curvature, random_seed, target_curvature_entrophy
  - state:
    - representation of centerline (one of these):
      - spline
      - Frenet Frame-based
      - raw Cartesian-coordinates points
  - output:
    - set of sampled centerline points (x, y, kappa, s)
    - kappa means curvautre, s means longitudinal distance traveld along the reference path (in Frenet coordinate system)
2. track(wall) generation
  - input: 
    - set of sampled centerline points (x, y, kappa, s)
  - output:
    - representation of track (one of these):
      - points
      - point cloud
      - probablistic distribution
      - signed distance function
3. air duct placement
  - input: *determined in previous step
  - output: *will discussed bellow

## output

- centerline
  - (x, y, kappa, s)
- track
  - (x, y, left_width, right_width)
    - note: here, (x, y) means centerline coordinates
  - or (x, y) points
- air duct placement
  - .png occupancy grid map
  - .yaml file that describe map properties
  - .csv file that describe air duct placement
  - transformation into .usd (universal scene descriptor) for nvidia issac-sim integration 

- .yaml map descriptor file:
  - width, height
  - generation parameter specification (like sampling point count, max_cuvature, random_seed, etc)
  - what algorithm used
  - start pos for make racecar able to drive.
    - (it need extra algorithm to find valid starting point with fittable space and angle)
    - also it should include initial speed (m/s) considering geometrical lookahead formation at that point.
  - image resolution, occupancy threshold etc.
  - (hopely) extra human-understadable track features: track difficulty

## main entrypoints

- main
  - get user input (or cli **args)
  - produce output.
  - also produce statistics, features, and visualization for debugging (optional)
- queueable call (as ROS2 node)
  - maintaining queue as length with given length argument
  - this ROS2 node needed, because another ROS2 node (Reinforcement eLearning trainer) should change map dynamically.

## features to consider

- Geometric & Meta-Properties: To move beyond "it looks like a track," we need metrics that quantify the difficulty and racing quality.
  - Functional Metrics
    - Tortuosity Index: The ratio of the actual track length to the displacement of its major features. Higher tortuosity implies a more technical, "tight" track.
    - Curvature Entropy (H_k): If you sample the curvature k(s) along the arc length s, a high entropy indicates a track with varied corner types (hairpins, sweepers, chicanes), preventing a simple "fixed-speed" strategy.
    - LIDAR Feature Density: Since F1Tenth relies on LIDAR, a "good" track avoids long, featureless straights that cause scan-matching drift. We can quantify this by the gradient of the distance to the nearest wall.
  - Meta-Properties for Competition
    - Overtaking Opportunity (O_i): Defined as segments where the track width W > 2 × W_car + buffer and the curvature is below a threshold.
    - Recovery Slack: The distance between the "optimal racing line" and the physical air duct. A competitive track should have "high-risk" zones where the racing line clips the ducts.

### linearity of each terms (idea)

- maximum curvature constraints -> barrier function
- self-avoidance -> potential field
- curvature entropy -> diminishing returns
- sector distribution (360 degree) -> centroid centering

- more considerable term strategies:
  - S-coordinates-based windowing
  - coupled term: some relation between curvature and track width limit could exists.
  - density vs entropy
  - oscillation of curvature?
