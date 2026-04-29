# 二维目标检测结果（YOLO + 深度对齐）
#
# 对应 Overview 中视觉感知节点的输出数据结构：
#   Detection: bbox, class, confidence, depth

uint16 x                        # bbox 左上角 x 像素坐标
uint16 y                        # bbox 左上角 y 像素坐标
uint16 width                    # bbox 宽度 (像素)
uint16 height                   # bbox 高度 (像素)
int16 class_id                  # 类别ID (0:人, 1:车, 2:自行车 等)
float32 confidence              # 置信度 [0.0, 1.0]
float32 depth                   # 目标深度距离 (米)，从深度图或双目视差获取
geometry_msgs/Point position    # 目标3D位置 (相机坐标系下，x=右, y=下, z=前)
