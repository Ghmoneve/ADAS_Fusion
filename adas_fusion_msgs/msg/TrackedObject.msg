# 融合跟踪目标
#
# 对应 Overview 中融合节点的输出：
#   TrackedObject: id, class, position, velocity, confidence, source_flag

int32 id                        # 跟踪目标ID (唯一且稳定)
int16 class_id                  # 目标类别ID (-1 表示未知)
geometry_msgs/Point position    # 目标位置 (base_link 坐标系，单位: m)
float32 vx                      # x 方向速度分量，m/s
float32 vy                      # y 方向速度分量，m/s
float32 confidence              # 综合置信度 [0.0, 1.0]
uint8 source_flag               # 来源标志位 (bit0: 视觉, bit1: 激光雷达, bit2: 毫米波雷达)
