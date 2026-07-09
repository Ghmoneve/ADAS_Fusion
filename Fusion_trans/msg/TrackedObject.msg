# 融合跟踪目标 (传感器无关)
#
# 由 FusionEngine.step() 输出, 不绑定任何特定传感器类型。

int32  id                       # 全局唯一跟踪 ID
float32[4] state                # [px, py, vx, vy] Kalman 状态
float32[4] covariance_diag      # P 矩阵对角线: [var(px), var(py), var(vx), var(vy)]
float32 confidence              # 综合置信度 [0, 1]
uint32  source_mask             # 来源传感器位掩码 (由 register_sensor 时分配的 bit 决定)
uint32  hit_count               # 累计命中帧数
uint32  miss_count              # 连续丢失帧数
string  status                  # CANDIDATE | CONFIRMED
