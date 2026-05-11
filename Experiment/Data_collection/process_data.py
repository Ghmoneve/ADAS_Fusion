#!/usr/bin/env python3
"""
process_data.py — 离线数据处理
===================
从 Jetson 采集的原始数据出发，生成：
  1. 毫米波雷达模拟数据 (radar.csv)
  2. 自车运动数据 (motion.csv, 从融合目标反推)
  3. 融合方差数据 (重写 fusion_variance 字段)

用法:
  python3 process_data.py --scene 1
  python3 process_data.py --scene 1 --sigma_pos 0.2 --sigma_bearing 2.0 --sigma_vel 0.1
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
RAW_DATA_DIR = SCRIPT_DIR / "raw_data"


def load_csv(path: Path):
    if not path.exists():
        return None, None
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return None, None
    timestamps = np.array([float(r["timestamp"]) for r in rows])
    data = {}
    for key in rows[0].keys():
        if key == "timestamp":
            continue
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (ValueError, KeyError):
                vals.append(np.nan)
        data[key] = np.array(vals)
    return timestamps, data


def write_csv(path: Path, timestamps, columns: dict):
    """columns: {col_name: np.array}"""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["timestamp"] + list(columns.keys())
        writer.writerow(header)
        for i in range(len(timestamps)):
            row = [f"{timestamps[i]:.6f}"]
            for col in columns.values():
                row.append(f"{col[i]:.6f}" if not np.isnan(col[i]) else "")
            writer.writerow(row)


def generate_radar(lidar_ts, lidar_data, sigma_pos, sigma_bearing_deg, sigma_vel, output_dir: Path):
    """
    从 LiDAR 前向距离反算毫米波雷达观测 + 高斯噪声。
    假设障碍物在正前方（bearing≈0°），且静止（v_radial=0）。
    场景 1~4 中障碍物都在自车正前方小角度范围内。

    Radar 测量模型:
      bearing ≈ 0° (正前方)  + 噪声
      distance = LiDAR 测距值  + 噪声
      radial_velocity ≈ 0 (静止障碍物)  + 噪声
    """
    dist_key = "distance_to_obstacle_m"
    if dist_key not in lidar_data:
        print("  [WARN] lidar.csv missing distance column, skipping radar generation")
        return

    sigma_bearing = math.radians(sigma_bearing_deg)
    n = len(lidar_ts)
    bearings = np.full(n, np.nan)
    distances = np.full(n, np.nan)
    velocities = np.full(n, np.nan)

    # 先对 LiDAR 做中值滤波，减少毛刺对雷达模拟的影响
    from scipy.ndimage import median_filter
    raw_dist = lidar_data[dist_key]
    try:
        smooth_dist = median_filter(raw_dist, size=11, mode='nearest')
    except Exception:
        smooth_dist = raw_dist  # fallback

    rng = np.random.RandomState(42)
    for i in range(n):
        true_dist = smooth_dist[i]
        if np.isnan(true_dist) or true_dist < 0.15 or true_dist > 10.0:
            continue
        # 真值: 障碍物在正前方
        true_bearing = 0.0
        true_vel = 0.0  # 静止障碍物
        # 加噪声
        noisy_dist = true_dist + rng.normal(0.0, sigma_pos)
        noisy_dist = max(0.15, noisy_dist)
        noisy_bearing = true_bearing + rng.normal(0.0, sigma_bearing)
        noisy_vel = true_vel + rng.normal(0.0, sigma_vel)

        bearings[i] = math.degrees(noisy_bearing)
        distances[i] = noisy_dist
        velocities[i] = noisy_vel

    write_csv(output_dir / "radar.csv", lidar_ts, {
        "bearing_deg": bearings,
        "distance_m": distances,
        "radial_velocity_ms": velocities,
    })
    print(f"  Generated radar.csv ({np.sum(~np.isnan(bearings))} valid rows, mean dist={np.nanmean(distances):.3f}m)")


def derive_motion(lidar_ts, lidar_data, fusion_ts, fusion_data, output_dir: Path):
    """
    反推自车运动。优先使用 fusion 跟踪的近处目标位置（卡尔曼已平滑），
    若 fusion 跟踪的是远处物体（>5m），则回退到 LiDAR 距离（大窗口平滑）。

    原理: 障碍物静止 → 自车位移 = 障碍物在自车系中位置的减少量。
    """
    from scipy.ndimage import median_filter

    # ── 判断数据源 ──
    px = fusion_data.get("fused_px")
    py = fusion_data.get("fused_py")
    use_fusion = False
    if px is not None and py is not None:
        fusion_dist = np.sqrt(px**2 + py**2)
        mean_fd = np.nanmean(fusion_dist)
        use_fusion = (0.3 < mean_fd < 5.0)  # 跟踪的是近处障碍物

    if use_fusion:
        # fusion 位置自带卡尔曼平滑，直接用
        smooth = fusion_dist.copy()
        ts = fusion_ts
        print(f"  Motion from fusion (mean dist={mean_fd:.2f}m)")
    else:
        # 回退到 LiDAR，大窗口平滑
        dist_key = "distance_to_obstacle_m"
        if dist_key not in lidar_data:
            print("  [WARN] No usable motion source, skipping")
            return
        raw_dist = lidar_data[dist_key].copy()
        kernel = 31  # ~3s 窗口，保留趋势、去除噪声
        smooth = median_filter(raw_dist, size=kernel, mode='nearest')
        # 剔除 >3m 的离群（FOV 切换）
        smooth = np.where(smooth > 3.0, np.nan, smooth)
        valid_idx = np.where(~np.isnan(smooth))[0]
        if len(valid_idx) < 2:
            print("  [WARN] Too few valid LiDAR points for motion")
            return
        smooth = np.interp(np.arange(len(smooth)), valid_idx, smooth[valid_idx])
        ts = lidar_ts
        print(f"  Motion from LiDAR (kernel=31, mean dist={np.nanmean(smooth):.2f}m)")

    n = len(smooth)

    # ── 位移: 以初始距离为参考 ──
    ref_dist = smooth[0]
    displacement = np.full(n, np.nan)
    for i in range(n):
        if not np.isnan(smooth[i]):
            displacement[i] = ref_dist - smooth[i]

    # ── 速度: 中心差分 ──
    velocity = np.full(n, np.nan)
    for i in range(1, n - 1):
        if not (np.isnan(displacement[i-1]) or np.isnan(displacement[i+1])):
            dt = ts[i+1] - ts[i-1]
            if dt > 0.001:
                velocity[i] = (displacement[i+1] - displacement[i-1]) / dt
    if n >= 2:
        dt0 = ts[1] - ts[0]
        if dt0 > 0.001:
            velocity[0] = (displacement[1] - displacement[0]) / dt0
        dt1 = ts[-1] - ts[-2]
        if dt1 > 0.001:
            velocity[-1] = (displacement[-1] - displacement[-2]) / dt1

    # ── 加速度: 中心差分 ──
    acceleration = np.full(n, np.nan)
    for i in range(1, n - 1):
        if not (np.isnan(velocity[i-1]) or np.isnan(velocity[i+1])):
            dt = ts[i+1] - ts[i-1]
            if dt > 0.001:
                acceleration[i] = (velocity[i+1] - velocity[i-1]) / dt
    if n >= 2:
        dt0 = ts[1] - ts[0]
        if dt0 > 0.001:
            acceleration[0] = (velocity[1] - velocity[0]) / dt0
        dt1 = ts[-1] - ts[-2]
        if dt1 > 0.001:
            acceleration[-1] = (velocity[-1] - velocity[-2]) / dt1

    # ── 物理限幅: 只砍掉计算噪声导致的超物理值 ──
    velocity = np.clip(velocity, -0.8, 0.8)
    acceleration = np.clip(acceleration, -6.0, 6.0)

    write_csv(output_dir / "motion.csv", ts, {
        "displacement_m": displacement,
        "velocity_ms": velocity,
        "acceleration_ms2": acceleration,
    })
    valid_n = np.sum(~np.isnan(displacement))
    mean_v = np.nanmean(np.abs(velocity[~np.isnan(velocity)]))
    print(f"  -> {valid_n} rows, mean|v|={mean_v:.3f} m/s")


def compute_fusion_variance(fusion_ts, fusion_data, lidar_ts, lidar_data, output_dir: Path):
    """
    计算传感器融合方差 (Algorithm.md §5.5.1):
      LiDAR:  σ²_lidar = σ²_l0 * (N_ref / |C|)
      雷达:  σ²_radar = σ²_r0 / (conf * (1 + α|v|/v₀))
      融合:  1/σ²_fused = 1/σ²_lidar + 1/σ²_radar  (精度叠加)

    将算出的 σ²_fused 写回 fusion.csv 的 fusion_variance 列。
    """
    px = fusion_data.get("fused_px")
    if px is None:
        return

    # 传感器噪声参数 (来自 fusion_params.yaml)
    sigma_lidar_0 = 0.03
    N_ref = 30
    cluster_size = 3       # 典型 LiDAR 聚类点数
    sigma_radar_0 = 0.2
    alpha = 0.5
    v0 = 10.0
    conf = 0.85              # 模拟雷达典型置信度

    sigma2_lidar = sigma_lidar_0**2 * (N_ref / cluster_size)
    sigma2_radar_base = sigma_radar_0**2 / conf

    n = len(fusion_ts)
    variance = np.full(n, np.nan)
    for i in range(n):
        vx = fusion_data.get("fused_vx", np.zeros(n))
        vy = fusion_data.get("fused_vy", np.zeros(n))
        v_radial = math.sqrt(vx[i]**2 + vy[i]**2)
        sigma2_radar = sigma2_radar_base / (1.0 + alpha * v_radial / v0)
        # 精度加权融合
        precision_lidar = 1.0 / max(sigma2_lidar, 1e-9)
        precision_radar = 1.0 / max(sigma2_radar, 1e-9)
        sigma2_fused = 1.0 / (precision_lidar + precision_radar)
        variance[i] = sigma2_fused

    # 重写 fusion.csv 含修正后的方差
    out_path = output_dir / "fusion.csv"
    orig_ts, orig_data = load_csv(out_path)
    if orig_ts is None:
        return
    orig_data["fusion_variance"] = variance
    write_csv(out_path, orig_ts, orig_data)
    mean_var = np.nanmean(variance)
    print(f"  Fusion variance computed: mean σ²_fused = {mean_var:.6f}")


def main():
    parser = argparse.ArgumentParser(description="Offline data processing for ADAS Fusion experiments")
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument("--sigma_pos", type=float, default=0.2, help="Radar distance noise std (m)")
    parser.add_argument("--sigma_bearing", type=float, default=2.0, help="Radar bearing noise std (deg)")
    parser.add_argument("--sigma_vel", type=float, default=0.1, help="Radar velocity noise std (m/s)")
    args = parser.parse_args()

    scene_dir = RAW_DATA_DIR / f"scene_{args.scene}"
    run_dir = scene_dir / "run_1"
    if not run_dir.exists():
        print(f"ERROR: Data directory not found: {run_dir}")
        sys.exit(1)

    print(f"Processing Scene {args.scene} data in {run_dir}")
    print(f"  Radar noise: σ_pos={args.sigma_pos}m, σ_bearing={args.sigma_bearing}°, σ_vel={args.sigma_vel}m/s")

    # Load raw data
    fusion_ts, fusion_data = load_csv(run_dir / "fusion.csv")
    lidar_ts, lidar_data = load_csv(run_dir / "lidar.csv")

    if fusion_ts is None:
        print("ERROR: fusion.csv not found or empty")
        sys.exit(1)

    # 1. Generate simulated radar data from LiDAR
    generate_radar(lidar_ts, lidar_data, args.sigma_pos, args.sigma_bearing, args.sigma_vel, run_dir)

    # 2. Derive motion from fusion or LiDAR
    derive_motion(lidar_ts, lidar_data, fusion_ts, fusion_data, run_dir)

    # 3. Compute fusion variance
    compute_fusion_variance(fusion_ts, fusion_data, lidar_ts, lidar_data, run_dir)

    print(f"\nDone. All data ready in {run_dir}/")
    print("Run the plot scripts next:")
    print(f"  cd Experiment/plot_py && python3 scene_{args.scene}_lidarPlot.py")
    print(f"  cd Experiment/plot_py && python3 scene_{args.scene}_radarPlot.py")
    print(f"  cd Experiment/plot_py && python3 scene_{args.scene}_TTCPlot.py")
    print(f"  cd Experiment/plot_py && python3 scene_{args.scene}_motionPlot.py")


if __name__ == "__main__":
    main()
