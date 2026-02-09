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
