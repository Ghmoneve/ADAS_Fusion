# 场景2：静态抛锚车避让（十字路口接入前） — 评价报告

> **实验日期**：YYYY-MM-DD  
> **运行编号**：run_X  
> **采集数据路径**：`Data_collection/raw_data/scene_2/run_X/`

---

## 1. TTC 分级准确性

| 风险等级 | 设定 TTC 范围 | 实测 TTC 范围 | 是否符合预期 |
|---------|-------------|-------------|:---------:|
| **SAFE**      | $> 5.0\ \mathrm{s}$ | — | — |
| **WARNING**   | .0 \sim 5.0\ \mathrm{s}$ | — | — |
| **SLOWDOWN**  | .0 \sim 3.0\ \mathrm{s}$ | — | — |
| **EMERGENCY** | $\le 1.0\ \mathrm{s}$ | — | — |

**TTC 分级准确性结论**：

---

## 2. 停车距离

| 指标 | 值 |
|-----|-----|
| 预警信号发出时刻 | XX.XXX s |
| 完全停车时刻 | XX.XXX s |
| 预警至停车行驶距离 | XX.XX m |
| 是否在碰撞前成功停车 | 是 / 否 |

**停车距离分析**：

---

## 3. 传感器方差分析

| 风险阶段 | $\sigma^2_{\mathrm{cam}}$ | $\sigma^2_{\mathrm{lidar}}$ | $\sigma^2_{\mathrm{radar}}$ | $\sigma^2_{\mathrm{fused}}$ |
|---------|--------------------------|----------------------------|----------------------------|---------------------------|
| SAFE      | — | — | — | — |
| WARNING   | — | — | — | — |
| SLOWDOWN  | — | — | — | — |
| EMERGENCY | — | — | — | — |

**融合方差变化趋势分析**：

---

## 4. 综合结论

- **融合算法表现**：
- **TTC 决策有效性**：
- **是否达到设计预期**：

---

## 5. 附图索引

| 图像 | 路径 |
|-----|------|
| LiDAR 距离图 | `Image/scene_2/scene_2_lidar.jpg` |
| 毫米波雷达图 | `Image/scene_2/scene_2_radar.jpg` |
| TTC 变化图 | `Image/scene_2/scene_2_TTC.jpg` |
| 运动状态图 | `Image/scene_2/scene_2_motion.jpg` |
| 相机关键帧 | `Data_collection/raw_data/scene_2/run_X/camera_keyframes/` |

---

*评价人：　　　　| 审核人：　　　　| 日期：*
