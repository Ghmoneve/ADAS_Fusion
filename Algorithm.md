# ADAS Fusion Algorithm

## 5.1 时间同步：滑动时间窗

**目的**：融合来自不同传感器（相机30Hz、雷达20Hz、激光雷达10Hz）的异步观测，使它们近似在同一时刻对齐。

**原理**：维护一个固定长度的时间窗口 $\Delta t$（默认 $100\,\text{ms}$）。当融合循环触发时（例如 $10\,\text{Hz}$），从每个传感器的历史缓存中取出时间戳满足下式的观测：

$$
t_{\text{now}} - \Delta t \le t_{\text{stamp}} \le t_{\text{now}}
$$

即窗口内所有观测都被视为“同步”。这种方法称为**近似时间同步**，简单且对短延时鲁棒。

**为什么可行？** 在低速机器人场景（$v \le 0.5\,\text{m/s}$）中，$100\,\text{ms}$ 内目标移动不超过 $5\,\text{cm}$，远小于传感器测量噪声，因此对融合精度影响可忽略。

---

## 5.2 空间对齐：TF坐标变换

每个传感器观测到的点 $\mathbf{p}_{\text{sensor}}$ 位于其自身坐标系下。通过机器人系统发布的静态TF变换（$R, t$），转换到统一参考系 `base_link`：

$$
\mathbf{p}_{\text{base}} = \mathbf{R}_{\text{sensor}}^{\text{base}} \cdot \mathbf{p}_{\text{sensor}} + \mathbf{t}_{\text{sensor}}^{\text{base}}
$$

速度向量仅需旋转：

$$
\mathbf{v}_{\text{base}} = \mathbf{R}_{\text{sensor}}^{\text{base}} \cdot \mathbf{v}_{\text{sensor}}
$$

**协方差变换**：若传感器坐标系下观测噪声协方差为 $\mathbf{R}_{\text{sensor}}$，则变换到 `base_link` 后的协方差为

$$
\mathbf{R}_{\text{base}} = \mathbf{R}_{\text{sensor}}^{\text{base}} \; \mathbf{R}_{\text{sensor}} \; \big(\mathbf{R}_{\text{sensor}}^{\text{base}}\big)^\top
$$

因其为线性变换，该传播公式严格成立（无近似）。本文后续讨论的传感器噪声模型均指在 `base_link` 坐标系下的协方差矩阵。

---

## 5.3 LiDAR 点云欧氏距离聚类

#### 5.3.1 欧氏距离定义
两点 $A(x_1,y_1)$ 和 $B(x_2,y_2)$ 之间的欧氏距离为：

$$
d_E(A,B) = \sqrt{(x_1-x_2)^2 + (y_1-y_2)^2}
$$

#### 5.3.2 聚类操作
算法流程（假设输入为二维点集 $P = \{\mathbf{p}_1, \mathbf{p}_2, ..., \mathbf{p}_N\}$）：
1. 对 $P$ 按角度排序（激光扫描天然有序）。
2. 初始化第一个点为一个新簇。
3. 遍历后续点 $\mathbf{p}_i$：
   - 若 $d_E(\mathbf{p}_i, \mathbf{p}_{i-1}) < \delta$（$\delta = 0.3\,\text{m}$），则将 $\mathbf{p}_i$ 加入当前簇；
   - 否则，结束当前簇，以 $\mathbf{p}_i$ 开始新簇。
4. 输出所有簇 $C_1, C_2, ..., C_M$。

**作用**：将离散的激光点云分组，每组对应一个可能的物理障碍物（例如行人、车辆）。

#### 5.3.3 聚类中心与置信度
每个簇的中心为：

$$
\mathbf{c} = \left( \frac{1}{|C|}\sum_{\mathbf{p}\in C} x,\ \frac{1}{|C|}\sum_{\mathbf{p}\in C} y \right)
$$

**置信度**定义为：

$$
\text{conf} = \min\left( \frac{|C|}{N_{\text{ref}}},\ 1.0 \right)
$$

其中 $N_{\text{ref}}=30$ 是经验参考值（一个典型障碍物在 $5\,\text{m}$ 处约产生30个点）。限幅操作 $\min(\cdot,1)$ 确保置信度不超过1。**意义**：点云越密，障碍物存在证据越充分，置信度越高。

**聚类中心协方差**：根据中心极限定理，$|C|$ 个大致独立同分布点云点的样本均值的协方差与 $|C|$ 成反比。因此 LiDAR 观测噪声方差可建模为

$$
\sigma_{\text{lidar}}^2 = \sigma_{\text{lidar,0}}^2 \frac{N_{\text{ref}}}{|C|}
$$

该模型将在 5.5 节用于推导最优融合权重。

---

## 5.4 数据关联与马氏距离检验

#### 5.4.1 为什么要做数据关联？
融合节点同时维护多个“跟踪目标”（如行人1、行人2）和当前帧的多个“观测”（来自三个传感器）。需要判断哪个观测属于哪个已存在的目标，或者是否为新目标。这就是**关联**问题。

#### 5.4.2 传感器观测模型
每个传感器 $s \in \{\text{cam},\ \text{lidar},\ \text{radar}\}$ 的观测 $\mathbf{z}_s \in \mathbb{R}^2$（目标在 `base_link` 下的位置）由真实位置 $\mathbf{x}_{\text{true}}$ 加噪声产生：

$$
\mathbf{z}_s = \mathbf{x}_{\text{true}} + \mathbf{v}_s, \qquad \mathbf{v}_s \sim \mathcal{N}(\mathbf{0},\ \mathbf{R}_s)
$$

其中 $\mathbf{R}_s \in \mathbb{R}^{2\times 2}$ 为传感器的**测量噪声协方差矩阵**。为简化而不过分丧失精度，假定各传感器的噪声各向同性，即 $\mathbf{R}_s = \sigma_s^2 \mathbf{I}_2$。具体方差模型 $\sigma_s^2$ 将在 5.5.1 节建立。

#### 5.4.3 马氏距离（Mahalanobis Distance）
欧氏距离假设各维度独立同分布，但实际中状态量的不同维度可能相关、且量纲不同。马氏距离考虑了数据分布的协方差结构，定义为：

$$
D_M(\mathbf{x}, \mathbf{y}) = \sqrt{(\mathbf{x}-\mathbf{y})^\top \mathbf{\Sigma}^{-1} (\mathbf{x}-\mathbf{y})}
$$

其中 $\mathbf{\Sigma}$ 是数据协方差矩阵。当 $\mathbf{\Sigma}=\mathbf{I}$ 时退化为欧氏距离。

#### 5.4.4 在关联中的应用
对于每个跟踪目标 $j$，卡尔曼滤波给出了**预测状态** $\mathbf{x}_{\text{pred}}^{(j)}$ 和**预测协方差** $\mathbf{P}_{\text{pred}}^{(j)}$。观测矩阵 $\mathbf{H}$ 提取位置：

$$
\mathbf{z}_{\text{pred}}^{(j)} = \mathbf{H} \mathbf{x}_{\text{pred}}^{(j)} = [p_x, p_y]^\top
$$

对于传感器 $s$ 的观测 $\mathbf{z}_s$，残差为：

$$
\mathbf{y}_{s}^{(j)} = \mathbf{z}_s - \mathbf{z}_{\text{pred}}^{(j)}
$$

残差的协方差矩阵（由预测不确定性 + 测量噪声构成）为：

$$
\mathbf{S}_{s}^{(j)} = \mathbf{H} \mathbf{P}_{\text{pred}}^{(j)} \mathbf{H}^\top + \mathbf{R}_s
$$

马氏距离平方为：

$$
D_M^2 = \big(\mathbf{y}_{s}^{(j)}\big)^\top \big(\mathbf{S}_{s}^{(j)}\big)^{-1} \mathbf{y}_{s}^{(j)}
$$

**假设检验**：记原假设 $H_0$：观测 $\mathbf{z}_s$ 来源于目标 $j$。在 $H_0$ 下，由于 $\mathbf{y}_s^{(j)} \sim \mathcal{N}(\mathbf{0}, \mathbf{S}_s^{(j)})$，有

$$
D_M^2 \sim \chi^2(2)
$$

取显著性水平 $\alpha = 0.05$，查表得阈值 $\chi^2_{0.95}(2) = 5.991$。若 $D_M^2 < 5.991$，则接受 $H_0$（认为观测属于该目标）；否则拒绝 $H_0$（观测可能是其他目标或噪声）。

#### 5.4.5 最优性证明
上述检验实为**似然比检验**。令 $L(\mathbf{z}_s \mid H_0) = \mathcal{N}(\mathbf{z}_{\text{pred}}, \mathbf{S}_s)$，备择假设下观测位置在监视区域内近似均匀分布。似然比统计量为

$$
\Lambda = \frac{L(\mathbf{z}_s \mid H_0)}{\sup_{H_1} L(\mathbf{z}_s \mid H_1)}
$$

取对数后，拒绝域 $\Lambda < c$ 等价于 $D_M^2 > \tau$。根据 **Neyman–Pearson 引理**，该检验在给定虚警概率 $\alpha$ 下使检测概率最大化。此外，在协方差 $\mathbf{S}_s$ 为已知的高斯分布下，它是 **一致最优无偏检验**。故马氏距离门限检测是关联环节在所述概率模型下的最优解。

#### 5.4.6 最近邻匹配
每个目标可能存在多个通过门限检验的候选观测。选择其中马氏距离平方 $D_M^2$ 最小的观测作为该目标的匹配观测（最近邻原则）。未被匹配的观测用于初始化新目标。该策略在稀疏场景下具有极低的关联错误率，结合最优门限构成完整的关联决策。

---

## 5.5 多传感器最优融合

融合节点对经过数据关联、确认属于同一目标的多个传感器观测进行融合，得到最优位置估计，用于后续卡尔曼状态更新。本节建立传感器噪声模型，并严格证明所采用的权重设计使融合结果在线性无偏估计类中达到最小方差。

### 5.5.1 传感器测量噪声模型

考虑任意传感器 $s$，其观测 $\mathbf{z}_s \in \mathbb{R}^2$ 的统计模型为

$$
\mathbb{E}[\mathbf{z}_s] = \mathbf{x}_{\text{true}}, \qquad \operatorname{Cov}(\mathbf{z}_s) = \mathbf{R}_s = \sigma_s^2 \mathbf{I}_2
$$

各传感器观测相互独立。根据各自物理特性及置信度信息，定义方差 $\sigma_s^2$ 如下：

1. **视觉（OAK‑D PRO 深度相机）**  
   双目深度估计误差近似与距离平方成正比，横向误差与距离成正比。为统一度量，采用有效噪声方差

   $$
   \sigma_{\text{cam}}^2 = \frac{\sigma_{\text{c},0}^2}{\text{conf}_{\text{cam}}} \exp\!\left(\frac{d^2}{2\sigma_c^2}\right)
   $$

   其中 $\sigma_{\text{c},0}$ 为近距离下单位置信度的基准标准差，$\sigma_c$ 为距离衰减尺度参数，$d$ 为目标径向距离，$\text{conf}_{\text{cam}}$ 为视觉检测置信度。该形式表明传感器精度（$1/\sigma^2$）随置信度增加而提高，随距离增加而指数衰减，与经典双目相机误差特性在远距离迅速退化的事实一致。

2. **激光雷达（RPLIDAR A1）**  
   由聚类中心协方差模型（§5.3.3），

   $$
   \sigma_{\text{lidar}}^2 = \sigma_{\text{l},0}^2 \frac{N_{\text{ref}}}{|C|}
   $$

   其中 $|C|$ 为聚类点数，$N_{\text{ref}}=30$。点云越密集，聚类中心越可靠，方差越小。

3. **毫米波雷达（MS60-3015S80M4）**  
   雷达的位置精度依赖于信噪比（置信度）以及多普勒径向速度产生的区分增益：

   $$
   \sigma_{\text{radar}}^2 = \frac{\sigma_{\text{r},0}^2}{\text{conf}_{\text{radar}} \big(1 + \alpha |v_{\text{radial}}|/v_0\big)}
   $$

   其中 $\alpha > 0$，$v_0 = 10\,\text{m/s}$ 为参考速度。径向速度越大，动态目标回波越强，定位越准，方差相应降低。

所有基准参数 $\sigma_{c,0}, \sigma_{l,0}, \sigma_{r,0}$ 及衰减常数 $\sigma_c, \alpha$ 可通过传感器标定实验确定。当某一传感器失效（如遮挡）时，其输出置信度接近零，方差趋于无穷，自动丧失对融合的贡献。

### 5.5.2 最优线性融合定理

**定理 5.1（最小方差线性无偏融合）**  
设 $\mathbf{z}_1,\dots,\mathbf{z}_m$ 是 $m$ 个来自不同传感器的独立观测，满足

$$
\mathbb{E}[\mathbf{z}_i] = \mathbf{x}, \quad \operatorname{Cov}(\mathbf{z}_i) = \sigma_i^2 \mathbf{I}_2,\quad i=1,\dots,m
$$

其中 $\mathbf{x} \in \mathbb{R}^2$ 为未知真实位置。定义线性融合估计

$$
\hat{\mathbf{x}} = \sum_{i=1}^m w_i \mathbf{z}_i, \qquad \sum_{i=1}^m w_i = 1
$$

则在均方误差 $J(\mathbf{w}) = \mathbb{E}\big[\|\hat{\mathbf{x}} - \mathbf{x}\|^2\big]$ 最小的意义下，最优权重为

$$
w_i^* = \frac{1/\sigma_i^2}{\sum_{j=1}^m 1/\sigma_j^2}, \qquad i=1,\dots,m
$$

相应的最小均方误差（即融合后方差）为

$$
\sigma_{\text{fused}}^2 = \frac{1}{\sum_{i=1}^m 1/\sigma_i^2}
$$

*证明*：  
由无偏性条件 $\sum w_i = 1$，得 $\hat{\mathbf{x}} - \mathbf{x} = \sum w_i (\mathbf{z}_i - \mathbf{x})$。因各观测独立，协方差为

$$
\operatorname{Cov}(\hat{\mathbf{x}}) = \sum_{i=1}^m w_i^2 \sigma_i^2 \mathbf{I}_2
$$

均方误差函数为 $J(\mathbf{w}) = \operatorname{tr} \operatorname{Cov}(\hat{\mathbf{x}}) = 2 \sum_{i=1}^m w_i^2 \sigma_i^2$。在约束 $\sum w_i = 1$ 下极小化 $J$ 等价于极小化 $\sum w_i^2 \sigma_i^2$。引入拉格朗日乘子 $\lambda$，定义

$$
\mathcal{L}(\mathbf{w}, \lambda) = \sum_{i=1}^m w_i^2 \sigma_i^2 - \lambda \left( \sum_{i=1}^m w_i - 1 \right)
$$

求偏导并令为零：

$$
\frac{\partial \mathcal{L}}{\partial w_i} = 2 w_i \sigma_i^2 - \lambda = 0 \;\Longrightarrow\; w_i = \frac{\lambda}{2\sigma_i^2}
$$

代入约束 $\sum w_i = (\lambda/2) \sum 1/\sigma_i^2 = 1$，得 $\lambda/2 = 1/\sum 1/\sigma_j^2$，故 $w_i^* = \frac{1/\sigma_i^2}{\sum 1/\sigma_j^2}$。此时

$$
\sum w_i^{*2} \sigma_i^2 = \frac{\sum (1/\sigma_i^4)\sigma_i^2}{\big(\sum 1/\sigma_j^2\big)^2} = \frac{1}{\sum 1/\sigma_i^2}
$$

因此 $\sigma_{\text{fused}}^2 = 1/\sum 1/\sigma_i^2$。证毕。

**注**：若噪声各向异性（$\mathbf{R}_i$ 非纯量阵），类似推导推广为矩阵权重 $\mathbf{W}_i$，最优解为 $\mathbf{W}_i = \big(\sum \mathbf{R}_j^{-1}\big)^{-1} \mathbf{R}_i^{-1}$，即著名的 **Gauss–Markov 定理**导致的最佳线性无偏估计。由于本文假设各向同性，标量权重即完全等价。

**推论 5.1（最大似然等价性）**  
在正态假设 $\mathbf{z}_i \sim \mathcal{N}(\mathbf{x}, \sigma_i^2 \mathbf{I})$ 下，上述融合估计亦是 $\mathbf{x}$ 的最大似然估计（MLE），其达到 Cramér–Rao 下界，是渐近有效的最优估计。

### 5.5.3 融合算法步骤

基于定理 5.1，设计融合权重如下：

1. 对于已关联到同一目标的所有观测 $i \in \mathcal{M}$，根据 5.5.1 节模型计算各自的测量方差 $\sigma_i^2$，并令精度 $\lambda_i = 1/\sigma_i^2$。
2. 归一化权重：
   $$
   w_i = \frac{\lambda_i}{\sum_{j \in \mathcal{M}} \lambda_j}
   $$
3. 融合位置：
   $$
   \mathbf{z}_{\text{fused}} = \sum_{i \in \mathcal{M}} w_i \mathbf{z}_i
   $$
4. 融合后的等效观测协方差：
   $$
   \mathbf{R}_{\text{fused}} = \sigma_{\text{fused}}^2 \mathbf{I}_2, \qquad \sigma_{\text{fused}}^2 = \frac{1}{\sum_{j \in \mathcal{M}} \lambda_j}
   $$

若某帧仅有一个传感器观测可用，则 $\mathbf{z}_{\text{fused}} = \mathbf{z}_i$，$\mathbf{R}_{\text{fused}} = \sigma_i^2 \mathbf{I}_2$。

该权重设计**严格**使得融合位置在给定的传感器噪声模型下达到最小方差，从而为最优解。与原有启发式权重（基于距离、置信度的指数调整）的本质精神一致，但在数学上更为精确：原有权重中的因子均映射为方差的倒数（精度），归一化后即得上述 $w_i$。因此，（稍作修正的）当前权重即最优融合权重。

### 5.5.4 与预测先验的融合——卡尔曼更新

在卡尔曼滤波框架中，预测状态 $\mathbf{x}_{\text{pred}}$ 可视为另一个独立的“先验观测”，其协方差为 $\mathbf{P}_{\text{pred}}$。融合观测 $\mathbf{z}_{\text{fused}}$ 的协方差为 $\mathbf{R}_{\text{fused}}$。后验状态的最优估计可由定理 5.1 直接推广，将 $\mathbf{x}_{\text{pred}}$ 纳入融合，或者等价地通过以下卡尔曼更新方程实现（两种方式严格等价）：

$$
\begin{aligned}
\mathbf{y} &= \mathbf{z}_{\text{fused}} - \mathbf{H}\mathbf{x}_{\text{pred}} \\
\mathbf{S} &= \mathbf{H}\mathbf{P}_{\text{pred}}\mathbf{H}^\top + \mathbf{R}_{\text{fused}} \\
\mathbf{K} &= \mathbf{P}_{\text{pred}}\mathbf{H}^\top \mathbf{S}^{-1} \\
\mathbf{x}_{\text{new}} &= \mathbf{x}_{\text{pred}} + \mathbf{K}\mathbf{y} \\
\mathbf{P}_{\text{new}} &= (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{\text{pred}}
\end{aligned}
$$

这个过程是预测先验与多传感器融合观测之间的**一步最优融合**，无需额外的指数平滑步骤。它既是线性最小方差估计，也是高斯假设下的最大后验估计（MAP）。因此，原先设计中独立的“先验‑后验平滑”步骤被移除，其功能已被卡尔曼更新所覆盖。

---

## 5.6 卡尔曼滤波（Kalman Filter）

> **本节符号说明**：
> - $\mathbf{x}$ : 状态向量，包含位置和速度，$\mathbf{x}=[p_x, p_y, v_x, v_y]^\top$
> - $\mathbf{F}$ : 状态转移矩阵（恒定速度模型）
> - $\Delta t$ : 离散时间步长（默认0.1秒）
> - $\mathbf{Q}$ : 过程噪声协方差矩阵
> - $q$ : 过程噪声系数（默认0.5）
> - $\mathbf{H}$ : 观测矩阵，提取位置 $[p_x, p_y]^\top$
> - $\mathbf{z}_k$ : 融合后的观测向量 $\mathbf{z}_{\text{fused}}$
> - $\mathbf{R}_{\text{fused},k}$ : 融合观测的噪声协方差矩阵，由 5.5.3 节计算
> - $\mathbf{P}$ : 状态协方差矩阵
> - $\mathbf{K}$ : 卡尔曼增益

#### 5.6.1 状态空间模型

状态向量 $\mathbf{x} = [p_x,\ p_y,\ v_x,\ v_y]^\top$。

**过程模型**（恒定速度假设）：
$$
\mathbf{x}_{k+1} = \mathbf{F} \mathbf{x}_k + \mathbf{w}_k,\quad \mathbf{w}_k \sim \mathcal{N}(0,\mathbf{Q})
$$
$$
\mathbf{F} = \begin{bmatrix}
1 & 0 & \Delta t & 0\\
0 & 1 & 0 & \Delta t\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

过程噪声协方差 $\mathbf{Q}$（加速度白噪声模型）：
$$
\mathbf{Q} = q \begin{bmatrix}
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0\\
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2}\\
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0\\
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2
\end{bmatrix}
$$

**观测模型**：使用 5.5 节生成的融合观测 $\mathbf{z}_{\text{fused}}$：
$$
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k,\quad \mathbf{v}_k \sim \mathcal{N}(0,\mathbf{R}_{\text{fused},k})
$$
$$
\mathbf{H} = \begin{bmatrix}1 & 0 & 0 & 0\\0 & 1 & 0 & 0\end{bmatrix}
$$
$\mathbf{R}_{\text{fused},k} = \sigma_{\text{fused},k}^2 \mathbf{I}_2$ 随各帧自适应变化，反映当前融合质量。

#### 5.6.2 滤波递推公式

**预测步骤**：
$$
\mathbf{x}_{\text{pred}} = \mathbf{F} \mathbf{x}_{\text{prev}}
$$
$$
\mathbf{P}_{\text{pred}} = \mathbf{F} \mathbf{P}_{\text{prev}} \mathbf{F}^\top + \mathbf{Q}
$$

**更新步骤**（与 5.5.4 一致）：
$$
\begin{aligned}
\mathbf{y} &= \mathbf{z}_{\text{fused}} - \mathbf{H} \mathbf{x}_{\text{pred}} \\
\mathbf{S} &= \mathbf{H} \mathbf{P}_{\text{pred}} \mathbf{H}^\top + \mathbf{R}_{\text{fused}} \\
\mathbf{K} &= \mathbf{P}_{\text{pred}} \mathbf{H}^\top \mathbf{S}^{-1} \\
\mathbf{x}_{\text{new}} &= \mathbf{x}_{\text{pred}} + \mathbf{K}\mathbf{y} \\
\mathbf{P}_{\text{new}} &= (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}_{\text{pred}}
\end{aligned}
$$

#### 5.6.3 全局最优性

**定理 5.2（高斯线性系统的最优估计）**  
考虑线性高斯状态空间模型，其中过程噪声与观测噪声均为零均值高斯白噪声，且各传感器观测通过 5.4 节的数据关联正确匹配。若时间同步误差可忽略，则按 5.5 节进行传感器间融合、再经上述卡尔曼滤波得到的状态估计 $\mathbf{x}_{\text{new}}$ 是 $\mathbf{x}_k$ 的**最小方差无偏估计（MVUE）**，同时也是最大后验估计（MAP）。  
*证明概要*：在正确关联下，融合观测 $\mathbf{z}_{\text{fused}}$ 的协方差 $\mathbf{R}_{\text{fused}}$ 按定理 5.1 达到同类线性融合的最小方差。将其输入标准卡尔曼滤波器，熟知该滤波器在线性高斯模型下生成状态的条件均值，即 MMSE 估计。由高斯分布性质，MMSE、MVUE、MAP 三者等价。因此整个流水线（观测→关联→融合→滤波）在所述假设下为全局最优。详细证明可参见 Anderson & Moore, *Optimal Filtering*。

---

> **文档版本**: v2.0  
> **最后更新**: 2026-05-01  
> **变更说明**: 针对融合与关联环节引入严格数学建模，证明权重设计的最优性，并用自适应观测协方差替代固定噪声参数，移除冗余平滑步骤，保证整体估计器在概率意义下的最优性。