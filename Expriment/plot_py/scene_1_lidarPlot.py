#!/usr/bin/env python3
"""Scene 1 LiDAR distance-to-obstacle plot."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import plot_lidar
plot_lidar(1)
