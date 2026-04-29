"""
ADAS Fusion -- Multi-sensor fusion and obstacle avoidance system.

Nodes:
  - detection_adapter: depthai SpatialDetectionArray → Detection2DArray
  - radar_adapter:     mmWave RadarTargetArray → RadarObjectArray (polar→Cartesian)
  - fusion_node:       multi-sensor adaptive fusion + Kalman filter tracking
  - decision_node:     TTC-based hierarchical obstacle avoidance
  - serial_bridge:     Jetson UART ↔ STM32F407 chassis controller
"""
