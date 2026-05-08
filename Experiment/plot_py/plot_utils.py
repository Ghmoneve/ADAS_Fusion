"""
Experiment plotting shared utilities.
Configures matplotlib with SimSun (Chinese) / Times New Roman (English & math),
risk-level background shading, axis formatting, and save helpers.
"""

import os
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ── Paths relative to this file ──────────────────────────────────────────
PLOT_PY_DIR = os.path.dirname(os.path.abspath(__file__))
EXPERIMENT_DIR = os.path.dirname(PLOT_PY_DIR)
RAW_DATA_DIR = os.path.join(EXPERIMENT_DIR, "Data_collection", "raw_data")
IMAGE_DIR = os.path.join(EXPERIMENT_DIR, "Image")

# ── Font & style setup ───────────────────────────────────────────────────
matplotlib.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "SimSun", "STSong", "serif"],
    "mathtext.fontset": "stix",
    "axes.titlesize": 13,
    "axes.labelsize": 11,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.1,
})

# ── TTC threshold constants ──────────────────────────────────────────────
TTC_SAFE = 5.0       # TTC > 5.0 → SAFE
TTC_WARNING = 3.0    # 3.0 < TTC ≤ 5.0 → WARNING
TTC_SLOWDOWN = 1.0   # 1.0 < TTC ≤ 3.0 → SLOWDOWN
TTC_EMERGENCY = 1.0  # TTC ≤ 1.0 → EMERGENCY

RISK_COLORS = {
    "SAFE":      "#4CAF50",   # green
    "WARNING":   "#FFC107",   # amber
    "SLOWDOWN":  "#FF9800",   # orange
    "EMERGENCY": "#F44336",   # red
    "RECOVERY":  "#2196F3",   # blue
}


def get_raw_data_path(scene: int) -> str:
    """Return path to raw_data/scene_<n>/ directory."""
    return os.path.join(RAW_DATA_DIR, f"scene_{scene}")


def get_image_path(scene: int, filename: str) -> str:
    """Return full path into Image/scene_<n>/<filename>."""
    out_dir = os.path.join(IMAGE_DIR, f"scene_{scene}")
    os.makedirs(out_dir, exist_ok=True)
    return os.path.join(out_dir, filename)


def get_image_path_root(filename: str) -> str:
    """Return full path into Image/ (root, not scene-specific)."""
    return os.path.join(IMAGE_DIR, filename)


def load_csv(scene: int, csv_name: str):
    """Load a CSV file from raw_data, returning (timestamps, data_dict).
    data_dict keys are the CSV column names beyond 'timestamp'.
    Each value is a 1-D numpy array.
    """
    import csv
    path = os.path.join(get_raw_data_path(scene), csv_name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Data file not found: {path}")

    with open(path, "r") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"Empty data file: {path}")

    timestamps = np.array([float(r["timestamp"]) for r in rows])
    data = {}
    for key in rows[0].keys():
        if key == "timestamp":
            continue
        data[key] = np.array([float(r[key]) for r in rows if r[key] != ""])
        # Handle possibly different length (empty cells)
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (ValueError, KeyError):
                vals.append(np.nan)
        data[key] = np.array(vals)

    return timestamps, data


def add_risk_backgrounds(ax, t_min: float, t_max: float):
    """Shade risk-level regions on the given axes with vertical spans.
    Uses TTC threshold lines to demark zones; callers pass the time-range
    and actual TTC data is NOT needed for the layout hints.

    Instead, this helper draws vertical divider lines at standard threshold
    positions; callers should pass the actual time boundaries per-scene.
    """
    # Semi-transparent band for EMERGENCY zone (left side)
    ax.axvspan(t_min, t_min + (t_max - t_min) * 0.15,
               alpha=0.08, color=RISK_COLORS["EMERGENCY"], label="EMERGENCY")
    ax.axvspan(t_min + (t_max - t_min) * 0.15, t_min + (t_max - t_min) * 0.40,
               alpha=0.08, color=RISK_COLORS["SLOWDOWN"], label="SLOWDOWN")
    ax.axvspan(t_min + (t_max - t_min) * 0.40, t_min + (t_max - t_min) * 0.70,
               alpha=0.08, color=RISK_COLORS["WARNING"], label="WARNING")
    ax.axvspan(t_min + (t_max - t_min) * 0.70, t_max,
               alpha=0.08, color=RISK_COLORS["SAFE"], label="SAFE")


def add_risk_backgrounds_by_ttc(ax, timestamps, ttc_values):
    """Shade risk regions using actual TTC data: classify each time point
    and draw continuous coloured bands."""
    if len(timestamps) == 0:
        return

    risk_levels = np.zeros_like(ttc_values, dtype=int)
    risk_levels[ttc_values > TTC_SAFE] = 0           # SAFE
    risk_levels[(ttc_values > TTC_WARNING) & (ttc_values <= TTC_SAFE)] = 1   # WARNING
    risk_levels[(ttc_values > TTC_SLOWDOWN) & (ttc_values <= TTC_WARNING)] = 2  # SLOWDOWN
    risk_levels[ttc_values <= TTC_SLOWDOWN] = 3       # EMERGENCY

    color_map = [
        RISK_COLORS["SAFE"],
        RISK_COLORS["WARNING"],
        RISK_COLORS["SLOWDOWN"],
        RISK_COLORS["EMERGENCY"],
    ]

    t_min, t_max = timestamps[0], timestamps[-1]
    for level in range(4):
        mask = risk_levels == level
        if not np.any(mask):
            continue
        # Find contiguous regions
        boundaries = np.diff(np.concatenate([[False], mask, [False]]).astype(int))
        starts = np.where(boundaries == 1)[0]
        ends = np.where(boundaries == -1)[0]
        for s, e in zip(starts, ends):
            t_start = timestamps[max(s - 1, 0)] if s > 0 else timestamps[s]
            t_end = timestamps[min(e, len(timestamps) - 1)]
            ax.axvspan(t_start, t_end, alpha=0.10, color=color_map[level])


def add_ttc_threshold_lines(ax, t_min: float, t_max: float):
    """Draw horizontal dashed lines at TTC=5.0, 3.0, 1.0 thresholds."""
    for val, label in [(TTC_SAFE, "TTC=5.0s"), (TTC_WARNING, "TTC=3.0s"),
                       (TTC_SLOWDOWN, "TTC=1.0s")]:
        ax.axhline(y=val, color="grey", linestyle="--", linewidth=0.8, alpha=0.7)
        ax.text(t_max, val, f"  {label}", fontsize=7, color="grey",
                verticalalignment="bottom", alpha=0.8)


def format_time_axis(ax, axis: str = "x"):
    """Format axis ticks to millisecond precision (or best available)."""
    if axis == "x":
        ax.xaxis.set_major_formatter(
            ticker.FuncFormatter(lambda v, _: f"{v:.3f}")
        )
    else:
        ax.yaxis.set_major_formatter(
            ticker.FuncFormatter(lambda v, _: f"{v:.3f}")
        )


def save_figure(fig, path: str):
    """Save figure at 300 dpi with tight bounding box."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=300, bbox_inches="tight", pad_inches=0.1)
    print(f"  Saved: {path}")


def format_sci_notation(val: float, precision: int = 3) -> str:
    """Format a float to scientific notation LaTeX string, e.g. $3.456\\times10^{-2}$."""
    if val == 0:
        return "$0$"
    exponent = int(np.floor(np.log10(abs(val))))
    mantissa = val / (10 ** exponent)
    return f"${mantissa:.{precision}f}\\times 10^{{{exponent}}}$"


# ═══════════════════════════════════════════════════════════════════════════
#  Core plotting functions — one per sensor/metric per scene
#  Each function: loads data → creates figure → adds risk backgrounds /
#  threshold lines / variance labels → saves to Image/<scene_n>/
# ═══════════════════════════════════════════════════════════════════════════

SCENE_TITLES = {
    1: "Scene 1: Constant-speed Following (Straight Road)",
    2: "Scene 2: Static Broken-down Vehicle (Intersection Approach)",
    3: "Scene 3: Lead Vehicle Sudden Braking (Curve Approach)",
    4: "Scene 4: Cross-intersection Lateral Cut-in",
    5: "Scene 5: Dark Environment — Constant-speed Following",
}


def _find_run_dir(scene: int):
    """Return the first run directory under raw_data/scene_<n>/."""
    scene_dir = os.path.join(RAW_DATA_DIR, f"scene_{scene}")
    if not os.path.isdir(scene_dir):
        return scene_dir  # return the expected dir even if not found
    # Look for run_1, run_2, etc. — use the first available
    run_dirs = sorted([
        d for d in os.listdir(scene_dir)
        if d.startswith("run_") and os.path.isdir(os.path.join(scene_dir, d))
    ])
    if run_dirs:
        return os.path.join(scene_dir, run_dirs[0])
    return scene_dir


def _load_csv_from_run(scene: int, csv_name: str):
    """Load CSV from a scene's run directory."""
    import csv as _csv
    data_dir = _find_run_dir(scene)
    path = os.path.join(data_dir, csv_name)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Data file not found: {path}")
    with open(path, "r") as f:
        reader = _csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError(f"Empty data file: {path}")
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
        if len(vals) == len(timestamps):
            data[key] = np.array(vals)
    return timestamps, data


# ── LiDAR Distance Plot ──────────────────────────────────────────────────
def plot_lidar(scene: int):
    print(f"Plotting LiDAR distance — Scene {scene} ...")
    timestamps, d = _load_csv_from_run(scene, "lidar.csv")

    fig, ax = plt.subplots(figsize=(10, 4.5))
    dist_key = "distance_to_obstacle_m"
    if dist_key not in d:
        raise KeyError(f"Expected column '{dist_key}' in lidar.csv")

    distance = d[dist_key]
    ax.plot(timestamps, distance, color="#1976D2", linewidth=1.2, label="LiDAR distance")

    # Risk backgrounds from TTC data if available
    try:
        t_ts, ttc_d = _load_csv_from_run(scene, "ttc.csv")
        if "ttc_value_s" in ttc_d:
            add_risk_backgrounds_by_ttc(ax, t_ts, ttc_d["ttc_value_s"])
    except (FileNotFoundError, KeyError):
        add_risk_backgrounds(ax, timestamps[0], timestamps[-1])

    # Annotate mean LiDAR variance
    sigma_lidar = np.nanstd(distance)
    ax.text(0.98, 0.92, f"$\\sigma_{{\\mathrm{{lidar}}}} = {sigma_lidar:.4f}$",
            transform=ax.transAxes, fontsize=9, ha="right", va="top",
            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to Obstacle (m)")
    ax.set_title(SCENE_TITLES.get(scene, f"Scene {scene}") + "\nLiDAR Distance-to-Obstacle")
    ax.legend(loc="upper right")
    format_time_axis(ax, "x")
    ax.grid(True, alpha=0.3)

    path = get_image_path(scene, f"scene_{scene}_lidar.jpg")
    save_figure(fig, path)
    plt.close(fig)


# ── Radar Plot (left-right layout: bearing | velocity) ───────────────────
def plot_radar(scene: int):
    print(f"Plotting Radar — Scene {scene} ...")
    timestamps, d = _load_csv_from_run(scene, "radar.csv")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 4.8))

    # Left: bearing angle
    if "bearing_deg" in d:
        ax1.plot(timestamps, d["bearing_deg"], color="#7B1FA2", linewidth=1.2, label="Bearing")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Bearing Angle (deg)")
    ax1.set_title("Target Bearing Angle")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    format_time_axis(ax1, "x")

    # Right: radial velocity
    if "radial_velocity_ms" in d:
        ax2.plot(timestamps, d["radial_velocity_ms"], color="#C2185B", linewidth=1.2, label="Radial Velocity")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Radial Velocity (m/s)")
    ax2.set_title("Target Radial Velocity")
    ax2.legend(loc="upper right")
    ax2.grid(True, alpha=0.3)
    format_time_axis(ax2, "x")

    sigma_radar = np.nanstd(d.get("radial_velocity_ms", [0]))
    ax2.text(0.98, 0.92, f"$\\sigma_{{\\mathrm{{radar}}}} = {sigma_radar:.4f}$",
             transform=ax2.transAxes, fontsize=9, ha="right", va="top",
             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))

    fig.suptitle(SCENE_TITLES.get(scene, f"Scene {scene}") + "\nMillimetre-wave Radar Observations",
                 fontsize=13, y=1.02)
    fig.tight_layout()

    path = get_image_path(scene, f"scene_{scene}_radar.jpg")
    save_figure(fig, path)
    plt.close(fig)


# ── TTC Plot ─────────────────────────────────────────────────────────────
def plot_ttc(scene: int):
    print(f"Plotting TTC — Scene {scene} ...")
    timestamps, d = _load_csv_from_run(scene, "ttc.csv")

    if "ttc_value_s" not in d:
        raise KeyError("Expected column 'ttc_value_s' in ttc.csv")

    ttc_vals = d["ttc_value_s"]
    fig, ax = plt.subplots(figsize=(10, 5))

    # Risk backgrounds from actual TTC data
    add_risk_backgrounds_by_ttc(ax, timestamps, ttc_vals)

    # Plot TTC curve
    ax.plot(timestamps, ttc_vals, color="#212121", linewidth=1.5, label="TTC")
    ax.fill_between(timestamps, 0, ttc_vals, alpha=0.08, color="#212121")

    # Threshold lines
    add_ttc_threshold_lines(ax, timestamps[0], timestamps[-1])

    # Risk level labels from data
    if "risk_level" in d:
        risk_str = [str(r) for r in d["risk_level"]]
        # Annotate phase transitions
        prev = None
        for i, r in enumerate(risk_str):
            if r != prev and r != "nan":
                ax.annotate(r, (timestamps[i], ttc_vals[i]),
                            textcoords="offset points", xytext=(0, 10),
                            fontsize=7, color=RISK_COLORS.get(r, "black"),
                            ha="center", fontweight="bold")
            prev = r

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("TTC (s)")
    ax.set_title(SCENE_TITLES.get(scene, f"Scene {scene}") + "\nTime-to-Collision (TTC)")
    ax.legend(loc="upper right")
    format_time_axis(ax, "x")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0)

    path = get_image_path(scene, f"scene_{scene}_TTC.jpg")
    save_figure(fig, path)
    plt.close(fig)


# ── Motion Plot (3-column layout: displacement | velocity | acceleration) ─
def plot_motion(scene: int):
    print(f"Plotting Motion — Scene {scene} ...")
    timestamps, d = _load_csv_from_run(scene, "motion.csv")

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 4.8))

    # Left: displacement
    if "displacement_m" in d:
        ax1.plot(timestamps, d["displacement_m"], color="#1B5E20", linewidth=1.2, label="Displacement")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Displacement (m)")
    ax1.set_title("Displacement")
    ax1.legend(loc="upper left")
    ax1.grid(True, alpha=0.3)
    format_time_axis(ax1, "x")

    # Middle: velocity
    if "velocity_ms" in d:
        ax2.plot(timestamps, d["velocity_ms"], color="#0D47A1", linewidth=1.2, label="Velocity")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.set_title("Velocity")
    ax2.legend(loc="upper left")
    ax2.grid(True, alpha=0.3)
    format_time_axis(ax2, "x")

    # Right: acceleration
    if "acceleration_ms2" in d:
        ax3.plot(timestamps, d["acceleration_ms2"], color="#B71C1C", linewidth=1.2, label="Acceleration")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Acceleration (m/s$^2$)")
    ax3.set_title("Acceleration")
    ax3.legend(loc="upper left")
    ax3.grid(True, alpha=0.3)
    format_time_axis(ax3, "x")

    fig.suptitle(SCENE_TITLES.get(scene, f"Scene {scene}") + "\nVehicle Motion Data",
                 fontsize=13, y=1.02)
    fig.tight_layout()

    path = get_image_path(scene, f"scene_{scene}_motion.jpg")
    save_figure(fig, path)
    plt.close(fig)


# ── Fusion Variance Comparison (cross-scene) ─────────────────────────────
def plot_fusion_variance(scenes=(1, 2, 3, 4, 5)):
    print(f"Plotting fusion variance comparison across scenes {scenes} ...")
    fig, ax = plt.subplots(figsize=(12, 5))
    colors = ["#1976D2", "#7B1FA2", "#C2185B", "#E64A19", "#00796B"]

    for idx, scene in enumerate(scenes):
        try:
            ts, d = _load_csv_from_run(scene, "fusion.csv")
            if "fusion_variance" in d:
                ax.plot(ts, d["fusion_variance"],
                        color=colors[idx % len(colors)], linewidth=1.2,
                        label=f"Scene {scene}", alpha=0.85)
        except (FileNotFoundError, KeyError):
            print(f"  [skip] Scene {scene} — fusion.csv not available")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Fusion Variance $\\sigma^2_{\\mathrm{fused}}$")
    ax.set_title("Fusion Variance Across Scenes")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    format_time_axis(ax, "x")

    path = get_image_path_root("fusion_variance_comparison.jpg")
    save_figure(fig, path)
    plt.close(fig)


# ── Scene 1 vs Scene 5 Comparison (light vs dark) ────────────────────────
def plot_scene_1_vs_5():
    print("Plotting Scene 1 vs Scene 5 comparison ...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    for row, scene in enumerate([1, 5]):
        label = "Light" if scene == 1 else "Dark"
        color = "#1976D2" if scene == 1 else "#F57C00"

        # TTC comparison
        ax_ttc = axes[row][0]
        try:
            ts, d = _load_csv_from_run(scene, "ttc.csv")
            if "ttc_value_s" in d:
                ax_ttc.plot(ts, d["ttc_value_s"], color=color, linewidth=1.3, label=f"Scene {scene} ({label})")
                add_risk_backgrounds_by_ttc(ax_ttc, ts, d["ttc_value_s"])
                add_ttc_threshold_lines(ax_ttc, ts[0], ts[-1])
        except (FileNotFoundError, KeyError):
            pass
        ax_ttc.set_title(f"TTC — Scene {scene} ({label})")
        ax_ttc.set_xlabel("Time (s)")
        ax_ttc.set_ylabel("TTC (s)")
        ax_ttc.legend(loc="upper right")
        ax_ttc.grid(True, alpha=0.3)
        format_time_axis(ax_ttc, "x")
        ax_ttc.set_ylim(bottom=0)

        # Fusion variance comparison
        ax_fus = axes[row][1]
        try:
            ts, d = _load_csv_from_run(scene, "fusion.csv")
            if "fusion_variance" in d:
                ax_fus.plot(ts, d["fusion_variance"], color=color, linewidth=1.3,
                            label=f"Scene {scene} ({label})")
        except (FileNotFoundError, KeyError):
            pass
        ax_fus.set_title(f"Fusion Variance — Scene {scene} ({label})")
        ax_fus.set_xlabel("Time (s)")
        ax_fus.set_ylabel("$\\sigma^2_{\\mathrm{fused}}$")
        ax_fus.legend(loc="upper right")
        ax_fus.grid(True, alpha=0.3)
        format_time_axis(ax_fus, "x")

    fig.suptitle("Scene 1 (Light) vs Scene 5 (Dark) — Fusion Algorithm Adaptability",
                 fontsize=14, y=1.01)
    fig.tight_layout()

    path = get_image_path_root("scene_1_vs_scene_5_comparison.jpg")
    save_figure(fig, path)
    plt.close(fig)
