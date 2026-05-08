#!/usr/bin/env python3
"""Scene 2 radar bearing / radial-velocity plot."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import plot_radar
plot_radar(2)
