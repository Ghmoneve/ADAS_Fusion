#!/usr/bin/env python3
"""Cross-scene fusion variance comparison plot."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from plot_utils import plot_fusion_variance
plot_fusion_variance()
