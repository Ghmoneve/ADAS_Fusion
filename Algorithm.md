# ADAS Fusion Algorithm

### 5.1 时间同步：滑动时间窗

**目的**：融合来自不同传感器（相机30Hz、雷达20Hz、激光雷达10Hz）的异步观测，使它们近似在同一时刻对齐。

**原理**：维护一个固定长度的时间窗口 $\Delta t$（默认 $100\,\text{ms}$）。当融合循环触发时（例如 $10\,\text{Hz}$），从每个传感器的历史缓存中取出时间戳满足下式的观测：

$$
t_{\text{now}} - \Delta t \le t_{\text{stamp}} \le t_{\text{now}}
$$

即窗口内所有观测都被视为“同步”。这种方法称为**近似时间同步**，简单且对短延时鲁棒。

**为什么可行？** 在低速机器人场景（$v \le 0.5\,\text{m/s}$）中，$100\,\text{ms}$ 内目标移动不超过 $5\,\text{cm}$，远小于传感器测量噪声，因此对融合精度影响可忽略。

### 5.2 空间对齐：TF坐标变换

每个传感器观测到的点 $\mathbf{p}_{\text{sensor}}$ 位于其自身坐标系下。通过机器人系统发布的静态TF变换（$R, t$），转换到统一参考系 `base_link`：

$$
\mathbf{p}_{\text{base}} = \mathbf{R}_{\text{sensor}}^{\text{base}} \cdot \mathbf{p}_{\text{sensor}} + \mathbf{t}_{\text{sensor}}^{\text{base}}
$$

速度向量仅需旋转：

$$
\mathbf{v}_{\text{base}} = \mathbf{R}_{\text{sensor}}^{\text{base}} \cdot \mathbf{v}_{\text{sensor}}
$$

### 5.3 LiDAR 点云欧氏距离聚类

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

### 5.4 数据关联与马氏距离检验

#### 5.4.1 为什么要做数据关联？
融合节点同时维护多个“跟踪目标”（如行人1、行人2）和当前帧的多个“观测”（来自三个传感器）。需要判断哪个观测属于哪个已存在的目标，或者是否为新目标。这就是**关联**问题。

#### 5.4.2 马氏距离（Mahalanobis Distance）
欧氏距离假设各维度独立同分布，但实际中状态量的不同维度可能相关、且量纲不同。马氏距离考虑了数据分布的协方差结构，定义为：

$$
D_M(\mathbf{x}, \mathbf{y}) = \sqrt{(\mathbf{x}-\mathbf{y})^T \mathbf{\Sigma}^{-1} (\mathbf{x}-\mathbf{y})}
$$

其中 $\mathbf{\Sigma}$ 是数据协方差矩阵。当 $\mathbf{\Sigma}=\mathbf{I}$ 时退化为欧氏距离。

#### 5.4.3 在关联中的应用
对于每个跟踪目标 $j$，卡尔曼滤波给出了**预测状态** $\mathbf{x}_{pred}^{(j)}$ 和**预测协方差** $\mathbf{P}_{pred}^{(j)}$。观测模型 $\mathbf{H}$ 提取位置：

$$
\mathbf{z}_{pred}^{(j)} = \mathbf{H} \mathbf{x}_{pred}^{(j)} = [p_x, p_y]^T
$$

观测残差（即观测与预测位置之差）：

$$
\mathbf{y} = \mathbf{z}_{obs} - \mathbf{z}_{pred}^{(j)}
$$

残差的协方差矩阵（称为残差协方差）为：

$$
\mathbf{S} = \mathbf{H} \mathbf{P}_{pred}^{(j)} \mathbf{H}^T + \mathbf{R}
$$

其中 $\mathbf{R}$ 是观测噪声协方差。那么马氏距离平方为：

$$
D_M^2 = \mathbf{y}^T \mathbf{S}^{-1} \mathbf{y}
$$

**假设检验**：若 $\mathbf{y} \sim \mathcal{N}(0,\mathbf{S})$，则 $D_M^2$ 服从自由度为 $\dim(\mathbf{z})=2$ 的卡方分布。取显著性水平 $\alpha=0.05$，查表得阈值 $\chi_{0.95}^2(2)=5.991$。如果 $D_M^2 < 5.991$，则认为该观测来源于该目标（接受原假设），否则拒绝（观测可能是其他目标或噪声）。

**好处**：相比固定欧氏距离门限，马氏距离能自动适应状态的不确定性（例如刚检测到的目标协方差大，门限自动放宽），提高关联鲁棒性。

#### 5.4.4 最近邻匹配
每个目标可能有多个候选观测满足马氏距离门限。选择其中 $D_M^2$ 最小的那个作为该目标的匹配观测（最近邻）。未被匹配的观测用于初始化新目标。

### 5.5 自适应贝叶斯融合

#### 5.5.1 贝叶斯融合思想
设目标真实状态为 $\mathbf{x}$（位置），多个传感器观测为 $\mathbf{z}_1,\mathbf{z}_2,\mathbf{z}_3$。由贝叶斯公式：

$$
p(\mathbf{x}|\mathbf{z}_1,\mathbf{z}_2,\mathbf{z}_3) \propto p(\mathbf{z}_1|\mathbf{x})\,p(\mathbf{z}_2|\mathbf{x})\,p(\mathbf{z}_3|\mathbf{x})\,p(\mathbf{x})
$$

假设各传感器独立且先验 $p(\mathbf{x})$ 均匀，则最大后验估计等价于最大化似然乘积。若似然均为高斯，则融合后的均值是各观测的加权平均，权重与精度（协方差逆）成正比。

#### 5.5.2 动态权重的设计
我们实际使用**加权平均**，权重设计原则：传感器置信度越高、越适合当前场景，权重越大。
- **视觉权重**：相机深度误差随距离指数增长 $\propto \exp(-d^2/(2\sigma_c^2))$  
  $$
  w_{cam} = w_{cam}^{base} \cdot conf_{cam} \cdot \exp\left(-\frac{d^2}{2\sigma_c^2}\right)
  $$
- **LiDAR权重**：点云密度越高，聚类越可靠  
  $$
  w_{lidar} = w_{lidar}^{base} \cdot \frac{|C|}{N_{ref}} \cdot \exp\left(-\frac{d^2}{2\sigma_l^2}\right)
  $$
- **雷达权重**：径向速度越大，动态目标优势越明显  
  $$
  w_{radar} = w_{radar}^{base} \cdot conf_{radar} \cdot \left(1 + \alpha \frac{|v_{radial}|}{10}\right)
  $$

归一化：
$$
\overline{w}_i = \frac{w_i}{\sum_{j} w_j}
$$

融合位置：
$$
\mathbf{z}_{fused} = \sum_i \overline{w}_i \cdot \mathbf{p}_i
$$

#### 5.5.3 先验-后验平滑
为避免融合结果因瞬间噪声跳变，与卡尔曼预测位置 $\mathbf{x}_{prior}$（即预测状态的前两维）作指数平滑：

$$
\alpha = \text{clip}\left( \frac{\sum \overline{w}_i}{3},\ 0.1,\ 0.9 \right)
$$
$$
\mathbf{z}_{final} = \alpha \mathbf{z}_{fused} + (1-\alpha) \mathbf{x}_{prior}
$$

当所有传感器都高可信时 $\alpha \approx 0.9$，信任测量；传感器集体失效时 $\alpha \to 0.1$，维持预测值。

### 5.6 卡尔曼滤波（Kalman Filter）

> **本节符号说明**：
> - $\mathbf{x}$ : 状态向量，包含位置和速度，$\mathbf{x}=[p_x, p_y, v_x, v_y]^T$
> - $\mathbf{F}$ : 状态转移矩阵，描述目标如何从上一时刻演化到当前时刻（恒定速度模型）
> - $\Delta t$ : 离散时间步长（与传感器融合周期一致，默认0.1秒）
> - $\mathbf{w}_k$ : 过程噪声（加速度随机扰动），假设为零均值高斯白噪声
> - $\mathbf{Q}$ : 过程噪声协方差矩阵，表示我们对运动模型不确定性的量化
> - $q$ : 过程噪声系数，控制加速度随机波动的强度（默认0.5）
> - $\mathbf{H}$ : 观测矩阵，将状态空间映射到观测空间（这里只提取位置）
> - $\mathbf{z}_k$ : 观测向量（来自自适应贝叶斯融合后的位置 $\mathbf{z}_{final}$）
> - $\mathbf{v}_k$ : 观测噪声，假设为零均值高斯白噪声
> - $\mathbf{R}$ : 观测噪声协方差矩阵，表示传感器测量的不确定度
> - $r$ : 观测噪声系数，控制位置测量的噪声方差（默认0.1）
> - $\mathbf{P}$ : 状态估计的协方差矩阵，表示当前状态估计的不确定性
> - $\mathbf{K}$ : 卡尔曼增益，决定观测对状态修正的权重
> - $\mathbf{I}$ : 单位矩阵
> - 下标 $pred$ : 表示预测值（先验），$new$ 表示更新后的值（后验）

#### 5.6.1 状态空间模型

状态向量 $\mathbf{x} = [p_x,\ p_y,\ v_x,\ v_y]^T$，其中：
- $p_x, p_y$ : 目标在平面上的位置（单位：米）
- $v_x, v_y$ : 目标沿X/Y轴的速度（单位：米/秒）

**过程模型**（恒定速度假设）：
$$
\mathbf{x}_{k+1} = \mathbf{F} \mathbf{x}_k + \mathbf{w}_k,\quad \mathbf{w}_k \sim \mathcal{N}(0,\mathbf{Q})
$$

状态转移矩阵 $\mathbf{F}$ 的具体形式：
$$
\mathbf{F} = \begin{bmatrix}
1 & 0 & \Delta t & 0\\
0 & 1 & 0 & \Delta t\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$
- 左上角 $2\times2$ 单位矩阵：位置自身传递
- 右上角 $2\times2$ 对角矩阵 $\Delta t$：速度乘以时间得到位置增量
- 右下角 $2\times2$ 单位矩阵：速度保持恒定（无外力）

**过程噪声协方差 $\mathbf{Q}$**（反映速度可能的变化，即加速度白噪声）：
$$
\mathbf{Q} = q \begin{bmatrix}
\frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2} & 0\\
0 & \frac{\Delta t^4}{4} & 0 & \frac{\Delta t^3}{2}\\
\frac{\Delta t^3}{2} & 0 & \Delta t^2 & 0\\
0 & \frac{\Delta t^3}{2} & 0 & \Delta t^2
\end{bmatrix}
$$
- $q$ ：过程噪声强度，值越大表示目标运动加速度变化越剧烈（我们取0.5）
- 推导依据：假设加速度为白噪声，对位置的影响与 $\Delta t^2/2$ 成正比，对速度的影响与 $\Delta t$ 成正比

**观测模型**（只观测位置，速度不可见）：
$$
\mathbf{z}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}_k,\quad \mathbf{v}_k \sim \mathcal{N}(0,\mathbf{R})
$$
观测矩阵 $\mathbf{H}$ 将四维状态投影到二维观测空间：
$$
\mathbf{H} = \begin{bmatrix}1 & 0 & 0 & 0\\0 & 1 & 0 & 0\end{bmatrix}
$$
- 乘法 $\mathbf{H} \mathbf{x}_k = [p_x, p_y]^T$ 提取位置

观测噪声协方差 $\mathbf{R}$ 为 $2\times2$ 对角矩阵：
$$
\mathbf{R} = r \begin{bmatrix}1 & 0\\0 & 1\end{bmatrix}
$$
- $r$ ：每个位置坐标的测量噪声方差（默认0.1，相当于标准差约0.316米）

#### 5.6.2 滤波递推公式

**预测步骤**（根据上一时刻最优估计，推算当前时刻的先验值）：
$$
\mathbf{x}_{pred} = \mathbf{F} \mathbf{x}_{prev}
$$
- $\mathbf{x}_{prev}$：上一帧修正后的状态（后验估计）
- $\mathbf{x}_{pred}$：预测的当前状态（先验估计）

$$
\mathbf{P}_{pred} = \mathbf{F} \mathbf{P}_{prev} \mathbf{F}^T + \mathbf{Q}
$$
- $\mathbf{P}_{prev}$：上一帧估计的协方差矩阵
- $\mathbf{P}_{pred}$：预测状态的协方差矩阵（不确定性）
- 物理意义：不确定性通过模型传递（$\mathbf{F}\mathbf{P}\mathbf{F}^T$），并叠加过程噪声（$\mathbf{Q}$）

**更新步骤**（利用当前观测修正预测）：

$$
\mathbf{y} = \mathbf{z}_{final} - \mathbf{H} \mathbf{x}_{pred}
$$
- $\mathbf{y}$：观测与预测位置之差（也称为残差），$2\times1$ 向量

$$
\mathbf{S} = \mathbf{H} \mathbf{P}_{pred} \mathbf{H}^T + \mathbf{R}
$$
- $\mathbf{S}$：残差协方差矩阵，$2\times2$，表示残差 $\mathbf{y}$ 的不确定性

$$
\mathbf{K} = \mathbf{P}_{pred} \mathbf{H}^T \mathbf{S}^{-1}
$$
- $\mathbf{K}$：卡尔曼增益，$4\times2$ 矩阵。它决定每个观测分量如何修正状态的不同维度
- 当观测噪声 $\mathbf{R}$ 相对较小（测量很准）时，$\mathbf{S}^{-1}$ 较大，$\mathbf{K}$ 较大，更信任观测
- 当预测协方差 $\mathbf{P}_{pred}$ 很小（模型很准）时，$\mathbf{K}$ 较小，更信任预测

$$
\mathbf{x}_{new} = \mathbf{x}_{pred} + \mathbf{K} \mathbf{y}
$$
- 最终状态估计（后验）：预测值加上观测修正量

$$
\mathbf{P}_{new} = (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_{pred}
$$
- 更新协方差矩阵，表示修正后不确定性减少的程度
- $\mathbf{I}$ 是 $4\times4$ 单位矩阵

**总结**：卡尔曼滤波通过预测-更新循环，不断融合带噪声的观测与运动模型，输出平滑、稳定的目标位置和速度估计。速度对于TTC计算至关重要（直接使用位置差分会放大噪声）。
