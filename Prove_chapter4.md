### 4.7 融合算法全局最优性证明

#### 4.7.1 融合观测与集中式观测的等价性

定理 4.1（等价性）：在假设A1‑A5下，按4.5.2节所得融合观测及其协方差进行的卡尔曼更新，与直接使用所有传感器原始观测的集中式卡尔曼更新，得到完全相同的后验状态估计和协方差矩阵。

证明：利用卡尔曼滤波的信息形式，...
（此处粘贴定理1的证明，并引用4.5.2中的权重公式）

该等价性表明，先融合再滤波的步骤没有任何信息损失。

#### 4.7.2 状态估计的最优性

由4.6.3节分析，线性高斯系统中的卡尔曼滤波器给出真实状态的条件均值，即最小均方误差估计（MMSE）、最大后验估计（MAP）和最小方差无偏估计（MVUE）。因此，集中式卡尔曼滤波在统计意义下最优。

#### 4.7.3 整体系统最优性

定理 4.2（全局最优性）：若数据关联正确（依据4.4.3节门限检验）、传感器观测独立且服从高斯分布、时间同步误差可忽略，则整个“关联 → 融合 → 卡尔曼滤波”流水线输出的状态估计是真实状态的最小方差无偏估计和最大后验估计。

证明：由4.4.3，关联为最优检验；由4.5.2，融合为最优线性无偏估计；由定理4.1，融合观测与集中观测等价；由4.6.3，卡尔曼滤波输出MMSE/MAP/MVUE。因此，流水线全局最优

# ADAS 多传感器融合系统全局最优性证明

## 1. 问题描述与假设

考虑一个动态目标在平面内的运动，系统通过以下步骤估计其状态：

1. **传感器观测**：三个异构传感器（相机、激光雷达、毫米波雷达）分别对目标位置产生独立的带噪观测；
2. **数据关联**：将各传感器观测与已存在的跟踪目标正确匹配；
3. **多传感器融合**：对属于同一目标的多个观测，使用最优线性无偏估计得到一个融合位置 $\mathbf{z}_{\text{fused}}$ 及其协方差矩阵 $\mathbf{R}_{\text{fused}}$；
4. **卡尔曼滤波**：将融合观测作为输入，进行状态预测与更新。

**全局最优性** 指：在上述假设下，该流水线输出的状态估计 $\mathbf{x}_{\text{new}}$ 是真实状态 $\mathbf{x}_k$ 的 **最小方差无偏估计** (MVUE) 以及 **最大后验估计** (MAP)，即达到了最小均方误差。

本证明严格建立在以下假设之上：

- (A1) 目标运动服从线性恒定速度模型，过程噪声为零均值高斯白噪声；
- (A2) 各传感器观测方程是真实位置的线性函数，观测噪声为零均值高斯白噪声，各传感器噪声相互独立；
- (A3) 数据关联完全正确，不存在误匹配；
- (A4) 时间同步误差可忽略，所有观测在同一离散时刻 $k$ 获得；
- (A5) 初始状态 $\mathbf{x}_0$ 服从已知的高斯分布（或确定已知），且与所有噪声独立。

符号沿用 Algorithm.md 文档中的约定，所有向量和矩阵的维度与定义均从该文档继承。

## 2. 预备知识与符号

$k$ 时刻真实状态向量为  

$$
\mathbf{x}_k = [p_x, p_y, v_x, v_y]^\top \in \mathbb{R}^4 .
$$

状态演化方程  

$$
\mathbf{x}_{k} = \mathbf{F} \mathbf{x}_{k-1} + \mathbf{w}_{k-1}, \quad \mathbf{w}_{k-1} \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}) ,
$$

其中 $\mathbf{F}$ 为状态转移矩阵（恒定速度模型），$\mathbf{Q}$ 为过程噪声协方差矩阵。

第 $i$ 个传感器（$i \in \{\text{cam}, \text{lidar}, \text{radar}\}$）提供的观测向量为 $\mathbf{z}^{(i)}_k \in \mathbb{R}^2$，满足  

$$
\mathbf{z}^{(i)}_k = \mathbf{H} \mathbf{x}_k + \mathbf{v}^{(i)}_k, \qquad \mathbf{v}^{(i)}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{R}^{(i)}_k),
$$

其中观测矩阵 $\mathbf{H} = [\mathbf{I}_2 \;|\; \mathbf{0}_{2 \times 2}]$ 仅提取位置分量。噪声 $\mathbf{v}^{(i)}_k$ 与过程噪声及不同传感器之间相互独立。测量噪声协方差矩阵记为 $\mathbf{R}^{(i)}_k = (\sigma^{(i)}_k)^2 \mathbf{I}_2$，具体数值由传感器噪声模型自适应给出。

定义 **集中观测向量**：将 $m$ 个传感器的观测垂直堆叠，得  

$$
\bar{\mathbf{z}}_k = \begin{bmatrix} \mathbf{z}^{(1)}_k \\ \vdots \\ \mathbf{z}^{(m)}_k \end{bmatrix} \in \mathbb{R}^{2m},
$$

其观测方程为  

$$
\bar{\mathbf{z}}_k = \bar{\mathbf{H}} \mathbf{x}_k + \bar{\mathbf{v}}_k, \quad \bar{\mathbf{H}} = \begin{bmatrix} \mathbf{H} \\ \vdots \\ \mathbf{H} \end{bmatrix} \in \mathbb{R}^{2m \times 4}, \quad \bar{\mathbf{v}}_k \sim \mathcal{N}(\mathbf{0}, \bar{\mathbf{R}}_k),
$$

其中 $\bar{\mathbf{R}}_k = \operatorname{diag}(\mathbf{R}^{(1)}_k, \dots, \mathbf{R}^{(m)}_k)$ 为块对角矩阵。

## 3. 多传感器融合的最优性（引理 1）

**引理 1**（最优线性融合）  
设 $\mathbf{z}^{(1)},\dots,\mathbf{z}^{(m)}$ 为 $m$ 个独立观测，$\mathbb{E}[\mathbf{z}^{(i)}] = \mathbf{x}$，$\operatorname{Cov}(\mathbf{z}^{(i)}) = (\sigma^{(i)})^2 \mathbf{I}_2$。  
在所有满足 $\sum w_i = 1$ 的线性估计 $\hat{\mathbf{x}} = \sum_i w_i \mathbf{z}^{(i)}$ 中，最小化均方误差 $J(\mathbf{w}) = \mathbb{E}\big[ \|\hat{\mathbf{x}} - \mathbf{x}\|^2 \big]$ 的权重为  

$$
w_i^* = \frac{\lambda_i}{\sum_{j} \lambda_j}, \qquad \lambda_i = 1/(\sigma^{(i)})^2 .
$$

相应的融合位置及其协方差为  

$$
\mathbf{z}_{\text{fused}} = \sum_i w_i^* \mathbf{z}^{(i)}, \qquad \mathbf{R}_{\text{fused}} = \frac{1}{\sum_i \lambda_i} \, \mathbf{I}_2 .
$$

*来源：* 该结果由拉格朗日乘子法求解约束优化问题得到，其本质上属于 Gauss‑Markov 定理的特殊情形（参见 [1, 第4章]）。在线性高斯模型下，该融合结果同时也是最大似然估计（MLE）和最小方差无偏估计（MVUE）[2, 第7章]。

## 4. 融合观测与集中式观测的等价性

本节证明：将融合观测 $\mathbf{z}_{\text{fused}}$ 连同协方差 $\mathbf{R}_{\text{fused}}$ 送入卡尔曼滤波器，所得的更新结果与直接使用原始集中观测 $\bar{\mathbf{z}}_k$ 进行卡尔曼更新完全等价。

**定理 1**（融合‑集中等价性）  
考虑线性高斯系统，设 $k$ 时刻预测状态为 $\mathbf{x}_{\text{pred}}$，预测协方差为 $\mathbf{P}_{\text{pred}}$。  
若按引理 1 构造 $\mathbf{z}_{\text{fused}}$ 与 $\mathbf{R}_{\text{fused}}$，则基于融合观测的卡尔曼更新：

$$
\begin{aligned}
\mathbf{y}   &= \mathbf{z}_{\text{fused}} - \mathbf{H} \mathbf{x}_{\text{pred}} \\
\mathbf{S}   &= \mathbf{H} \mathbf{P}_{\text{pred}} \mathbf{H}^\top + \mathbf{R}_{\text{fused}} \\
\mathbf{K}   &= \mathbf{P}_{\text{pred}} \mathbf{H}^\top \mathbf{S}^{-1} \\
\mathbf{x}_{\text{new}} &= \mathbf{x}_{\text{pred}} + \mathbf{K} \mathbf{y} \\
\mathbf{P}_{\text{new}} &= (\mathbf{I} - \mathbf{K} \mathbf{H}) \mathbf{P}_{\text{pred}}
\end{aligned}
$$

与直接使用集中观测 $\bar{\mathbf{z}}_k$ 的标准卡尔曼更新：

$$
\begin{aligned}
\bar{\mathbf{y}}   &= \bar{\mathbf{z}}_k - \bar{\mathbf{H}} \mathbf{x}_{\text{pred}} \\
\bar{\mathbf{S}}   &= \bar{\mathbf{H}} \mathbf{P}_{\text{pred}} \bar{\mathbf{H}}^\top + \bar{\mathbf{R}}_k \\
\bar{\mathbf{K}}   &= \mathbf{P}_{\text{pred}} \bar{\mathbf{H}}^\top \bar{\mathbf{S}}^{-1} \\
\bar{\mathbf{x}}_{\text{new}} &= \mathbf{x}_{\text{pred}} + \bar{\mathbf{K}} \bar{\mathbf{y}} \\
\bar{\mathbf{P}}_{\text{new}} &= (\mathbf{I} - \bar{\mathbf{K}} \bar{\mathbf{H}}) \mathbf{P}_{\text{pred}}
\end{aligned}
$$

产生完全相同的后验状态估计和协方差矩阵：

$$
\mathbf{x}_{\text{new}} = \bar{\mathbf{x}}_{\text{new}}, \qquad \mathbf{P}_{\text{new}} = \bar{\mathbf{P}}_{\text{new}} .
$$

*证明*：  
使用卡尔曼滤波的信息形式（见 [1, 第5章]），后验信息矩阵（协方差之逆）的更新为  

$$
\mathbf{P}_{k|k}^{-1} = \mathbf{P}_{k|k-1}^{-1} + \mathbf{H}_k^\top \mathbf{R}_k^{-1} \mathbf{H}_k .
$$

对于集中观测，观测矩阵为 $\bar{\mathbf{H}}$，噪声协方差为 $\bar{\mathbf{R}}_k$，其信息矩阵增量为  

$$
\bar{\mathbf{H}}^\top \bar{\mathbf{R}}_k^{-1} \bar{\mathbf{H}} = \sum_{i=1}^m \mathbf{H}^\top (\mathbf{R}^{(i)}_k)^{-1} \mathbf{H} .
$$

对于融合观测，其等价噪声协方差满足 $\mathbf{R}_{\text{fused}}^{-1} = \sum_i (\mathbf{R}^{(i)}_k)^{-1}$（由引理 1），故  

$$
\mathbf{H}^\top \mathbf{R}_{\text{fused}}^{-1} \mathbf{H} = \mathbf{H}^\top \Big( \sum_i (\mathbf{R}^{(i)}_k)^{-1} \Big) \mathbf{H} = \bar{\mathbf{H}}^\top \bar{\mathbf{R}}_k^{-1} \bar{\mathbf{H}} .
$$

因此两种更新产生的后验信息矩阵相等：

$$
\mathbf{P}_{\text{new}}^{-1} = \mathbf{P}_{\text{pred}}^{-1} + \mathbf{H}^\top \mathbf{R}_{\text{fused}}^{-1} \mathbf{H} = \mathbf{P}_{\text{pred}}^{-1} + \bar{\mathbf{H}}^\top \bar{\mathbf{R}}_k^{-1} \bar{\mathbf{H}} = \bar{\mathbf{P}}_{\text{new}}^{-1} .
$$

进而后验协方差矩阵相等：$\mathbf{P}_{\text{new}} = \bar{\mathbf{P}}_{\text{new}}$。

后验均值的更新可写为信息向量形式 [1]：

$$
\mathbf{P}_{k|k}^{-1} \mathbf{x}_{k|k} = \mathbf{P}_{k|k-1}^{-1} \mathbf{x}_{k|k-1} + \mathbf{H}_k^\top \mathbf{R}_k^{-1} \mathbf{z}_k .
$$

对融合观测：

$$
\mathbf{P}_{\text{new}}^{-1} \mathbf{x}_{\text{new}} = \mathbf{P}_{\text{pred}}^{-1} \mathbf{x}_{\text{pred}} + \mathbf{H}^\top \mathbf{R}_{\text{fused}}^{-1} \mathbf{z}_{\text{fused}} .
$$

计算右端第二项：

$$
\begin{aligned}
\mathbf{H}^\top \mathbf{R}_{\text{fused}}^{-1} \mathbf{z}_{\text{fused}}
&= \mathbf{H}^\top \Big( \sum_j (\mathbf{R}^{(j)}_k)^{-1} \Big) \sum_i \frac{(\mathbf{R}^{(i)}_k)^{-1}}{\sum_j (\mathbf{R}^{(j)}_k)^{-1}} \mathbf{z}^{(i)}_k \\
&= \mathbf{H}^\top \sum_i (\mathbf{R}^{(i)}_k)^{-1} \mathbf{z}^{(i)}_k \\
&= \sum_i \mathbf{H}^\top (\mathbf{R}^{(i)}_k)^{-1} \mathbf{z}^{(i)}_k \\
&= \bar{\mathbf{H}}^\top \bar{\mathbf{R}}_k^{-1} \bar{\mathbf{z}}_k .
\end{aligned}
$$

因此，两种更新所得的右端信息向量相等。结合信息矩阵已证相等，即得 $\mathbf{x}_{\text{new}} = \bar{\mathbf{x}}_{\text{new}}$。 ∎

**推论**：将多传感器观测预先融合成单个等效观测 $\mathbf{z}_{\text{fused}}$ 并执行卡尔曼更新，与直接使用所有原始观测的集中式卡尔曼滤波器在数学上完全等价，**没有任何信息损失**。

## 5. 卡尔曼滤波的最优性（引理 2）

**引理 2**（卡尔曼滤波最优性）  
在满足假设 (A1)–(A5) 的线性高斯状态空间模型中，卡尔曼滤波器给出的状态估计 $\mathbf{x}_{k|k}$ 是真实状态 $\mathbf{x}_k$ 在给定所有历史与当前观测 $\mathbf{Z}_k = \{\mathbf{z}_1, \dots, \mathbf{z}_k\}$ 下的条件均值：

$$
\mathbf{x}_{k|k} = \mathbb{E}[ \mathbf{x}_k \mid \mathbf{Z}_k ] .
$$

由高斯分布的性质，条件均值也是最小均方误差估计 (MMSE)、最大后验估计 (MAP) 以及最小方差无偏估计 (MVUE) [1][2]。

*来源：* 此为卡尔曼滤波的核心性质，详细证明见 Anderson & Moore [1, 第4‑5章] 或 Bar‑Shalom 等 [3, 第5章]。

## 6. 流水线全局最优性

综合前文引理，我们给出系统全局最优性定理。

**定理 2**（流水线全局最优性）  
给定假设 (A1)–(A5)，执行以下步骤：

1. 通过 5.4 节的数据关联正确地将各传感器观测分配到对应目标；
2. 对每个目标的观测集合，依据引理 1 计算融合位置 $\mathbf{z}_{\text{fused}}$ 及协方差 $\mathbf{R}_{\text{fused}}$；
3. 将融合结果作为观测，按卡尔曼滤波递推（预测 + 更新）得到状态估计 $\mathbf{x}_{\text{new}}$。

则 $\mathbf{x}_{\text{new}}$ 是真实状态 $\mathbf{x}_k$ 的 MVUE 和 MAP 估计，即达到全局最小均方误差。

*证明*：  
由数据关联正确，可独立处理每个目标而不失一般性。

对第 $k$ 时刻，已正确获得属于该目标的所有传感器观测。根据 **定理 1**，融合观测 $\mathbf{z}_{\text{fused}}$ 与协方差 $\mathbf{R}_{\text{fused}}$ 所产生的卡尔曼更新，与直接使用全部原始观测的集中式卡尔曼更新等价。因此，本流水线的融合‑滤波步骤等效于标准集中式卡尔曼滤波器。

依据 **引理 2**，标准卡尔曼滤波器在线性高斯模型下给出状态的条件均值，即 MMSE 估计，且该估计也是 MAP 和 MVUE。因此，流水线输出的 $\mathbf{x}_{\text{new}}$ 即为 $\mathbf{x}_k$ 的最优状态估计。

若某时刻仅有部分传感器（或单个）提供观测，融合退化为直接使用该观测及其协方差，上述等价性与最优性依然成立。 ∎

## 7. 结论

本文档从概率论与估计理论出发，严格证明了 ADAS 融合系统中“观测关联 → 多传感器融合 → 卡尔曼滤波”流水线在给定假设下输出目标状态的全局最优估计。具体地：

- 数据关联采用基于马氏距离的似然比检验，其最优性由 Neyman–Pearson 引理保证（见 Algorithm.md §5.4.5）；
- 多传感器融合采用精度加权的最优线性无偏估计（引理 1），且证明了该融合与集中式观测等价，无信息损失（定理 1）；
- 卡尔曼滤波本身在线性高斯模型下提供条件均值，从而实现 MMSE/MAP/MVUE 估计（引理 2）。

因此，整个系统设计具有坚实的理论保证，并非启发式构造。

---

### 参考文献

[1] B. D. O. Anderson, J. B. Moore, *Optimal Filtering*, Prentice-Hall, 1979.  
[2] S. M. Kay, *Fundamentals of Statistical Signal Processing: Estimation Theory*, Prentice-Hall, 1993.  
[3] Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001.
