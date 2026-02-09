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

- 1. generate initial random points (boundary from `width`, `height`, and number of points from `n_points`)
- 2. run Lloyd's algorithm (`max_lloyd_iterations` times) to pseudo-normalizing the spacing between points.
- 3. select random region indices from generated voronoi diagram by virtual_grid. (with coverage `virtual_grid_coverage`)
- 4. get concave hull forming polygon that connects given regions' centroids.
- 5. based on polygon, select interpolating vertex from voronoi diagram, considering `curvature_threshold` and non-crossing validation.
- 6. with given polygon, interpolate it, make it centerline

### 4.2 wall generation

- 1. with given centerline, get metadata for each points (parameter t, tangent vector, curvature, distance along curve, left raycast length, right raycast length)
  - raycast length mean, try to find first raycasted point from its polygon, where raycast origin point is the given point, and direction is perpendicular to its tangent vector, so left and right.
- 2. given custom lateral offset function with function signature contains parameters as mentioned above, generate point clouds for each points of centerline.
- 3. given point cloud, generate wall curves with DBSCAN
- 4. give duct thickness to wall curves.
- 5. summary and get metadata of track. (generated centerline (each point includes coordinates, cumulative distance along centerline for Frenet coordinates, curvature, left width, right width), wall occupancy grid, (TODO:optimzed global path))


### 4.3 global path optimization

## 5. Output format

- `centerline.npy` : centerline.py에서 생성된 센터라인 포인트 (x, y, t, cumulative_distance, curvature, left_width, right_width) 배열 (N x 7)
- `wall.npy` : wall.py에서 생성된 벽 occupancy grid (w x h) (resolution 고려)
- `optimized_global_path.npy` : optimized_global_path.py에서 생성된 최적화된 전체 경로 (x, y, t, curvature, left_width, right_width) 배열 (N x 7)

## 6. Example and usage

## 7. Troubleshooting?

## 8. References

- https://github.com/mvanlobensels/random-track-generator
- https://en.wikipedia.org/wiki/Voronoi_diagram
- https://en.wikipedia.org/wiki/Lloyd%27s_algorithm
- https://en.wikipedia.org/wiki/DBSCAN
- assisted by llm: deepseek
- https://github.com/duhaime/lloyd/tree/master
- https://github.com/cubao/concave_hull
