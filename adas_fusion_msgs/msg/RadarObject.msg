# 毫米波雷达目标 (笛卡尔坐标系)
#
# 由 radar_adapter 将原始 RadarTarget (极坐标) 转换而来

geometry_msgs/Point position     # 笛卡尔坐标 (x, y, z=0)，单位: m
float32 vx                       # x 方向速度分量，m/s
float32 vy                       # y 方向速度分量，m/s
float32 confidence               # 置信度 [0.0, 1.0]
