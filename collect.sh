#!/bin/bash
# collect.sh — Launch ADAS Fusion experiment data collection.
#
# Usage:
#   ./collect.sh --scene <1-5> --run <run_id> [--duration <sec>] [--odom_topic <name>] [--output_dir <path>]
#
# Examples:
#   ./collect.sh --scene 1 --run 1
#   ./collect.sh --scene 3 --run 2 --duration 180
#   ./collect.sh --scene 5 --run 1 --odom_topic /turtlebot4/odom

set -euo pipefail

# ── Defaults ─────────────────────────────────────────────────────────────
SCENE=""
RUN_ID=""
DURATION=120
ODOM_TOPIC="/odom"
OUTPUT_DIR=""  # auto-derived if empty

# ── Parse arguments ──────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --scene)
            SCENE="$2"; shift 2 ;;
        --run)
            RUN_ID="$2"; shift 2 ;;
        --duration)
            DURATION="$2"; shift 2 ;;
        --odom_topic)
            ODOM_TOPIC="$2"; shift 2 ;;
        --output_dir)
            OUTPUT_DIR="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 --scene <1-5> --run <id> [--duration <sec>] [--odom_topic <name>]"
            exit 0 ;;
        *)
            echo "Unknown argument: $1"
            exit 1 ;;
    esac
done

# ── Validate required args ───────────────────────────────────────────────
if [[ -z "$SCENE" || -z "$RUN_ID" ]]; then
    echo "ERROR: --scene and --run are required."
    echo "Usage: $0 --scene <1-5> --run <id>"
    exit 1
fi

if ! [[ "$SCENE" =~ ^[1-5]$ ]]; then
    echo "ERROR: --scene must be 1–5."
    exit 1
fi

# ── Locate project root ──────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

echo "══════════════════════════════════════════════"
echo "  ADAS Fusion — Experiment Data Collection"
echo "  Scene:       $SCENE"
echo "  Run ID:      $RUN_ID"
echo "  Duration:    ${DURATION}s"
echo "  Odom topic:  $ODOM_TOPIC"
echo "  Output:      Experiment/Data_collection/raw_data/scene_${SCENE}/run_${RUN_ID}/"
echo "══════════════════════════════════════════════"

# ── Source ROS2 environment ──────────────────────────────────────────────
if [[ -z "${ROS_DISTRO:-}" ]]; then
    if [[ -f /opt/ros/humble/setup.bash ]]; then
        source /opt/ros/humble/setup.bash
    elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
        source /opt/ros/jazzy/setup.bash
    else
        echo "ERROR: ROS2 not sourced and default setup.bash not found."
        echo "  Please source your ROS2 environment first."
        exit 1
    fi
fi

# ── Source workspace if install exists ───────────────────────────────────
if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi

# ── Run collector ────────────────────────────────────────────────────────
COLLECTOR_SCRIPT="$PROJECT_ROOT/Experiment/Data_collection/data_collector.py"

if [[ ! -f "$COLLECTOR_SCRIPT" ]]; then
    echo "ERROR: collector script not found at $COLLECTOR_SCRIPT"
    exit 1
fi

echo "[collect.sh] Starting data collector..."
python3 "$COLLECTOR_SCRIPT" \
    --scene "$SCENE" \
    --run "$RUN_ID" \
    --duration "$DURATION" \
    --odom_topic "$ODOM_TOPIC"

echo "[collect.sh] Collection complete."
echo "[collect.sh] Data saved to: Experiment/Data_collection/raw_data/scene_${SCENE}/run_${RUN_ID}/"
