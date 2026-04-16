# MATLAB 项目：多无人机协同搜索与动态目标跟踪

这是一个可以直接运行的 MATLAB 工程，不是单段脚本。

它展示了 4 层能力：

1. **多智能体协同控制**：多架无人机自动分工，一部分负责搜索，一部分负责跟踪。
2. **动态系统建模**：目标会在二维空间内运动、转向、碰撞反弹。
3. **状态估计**：使用 **Kalman Filter** 对目标进行预测与修正，而不是“开天眼”。
4. **工程化可视化**：包含实时动画、任务分配线、覆盖热图、任务总结面板。

---

## 运行方法

把整个文件夹放到 MATLAB 当前工作目录下，然后直接运行：

```matlab
run_swarm_demo
```

---

## 文件说明

- `run_swarm_demo.m`：主入口，一键运行
- `defaultMissionConfig.m`：参数配置
- `createScenario.m`：随机生成场景、障碍物、初始智能体与目标
- `simulateSwarmMission.m`：核心仿真主循环
- `updateTargets.m`：动态目标运动模型
- `assignTargets.m`：任务分配逻辑
- `selectCoverageWaypoints.m`：搜索航点选择
- `computeAgentVelocity.m`：控制律（目标吸引 + 避障 + 群体分离）
- `renderMissionFrame.m`：实时动画渲染
- `plotMissionSummary.m`：后处理总结图

---

## 项目亮点

### 1) 不是“预先写死路径”的动画
无人机不是沿着固定轨迹飞，而是会根据：
- 当前覆盖热图
- 目标是否被观测到
- 当前估计位置
- 障碍物位置
- 其他无人机位置

实时改变自己的行为。

### 2) 目标跟踪不是作弊
只有当目标进入传感器范围，系统才会得到带噪声的位置测量；
平时靠卡尔曼滤波做预测。

### 3) 搜索与跟踪自动切换
系统会让部分无人机继续扩大覆盖范围，另一部分自动转为追踪模式。
这比单纯的“大家一起追最近点”更像一个真正的小型任务系统。

### 4) 容易继续魔改
你可以继续往上叠：
- A* / RRT 路径规划
- MPC 轨迹控制
- EKF / UKF
- 强化学习策略
- 编队控制
- 3D 空域模型
- GUI App Designer 控制台

---

## 建议你怎么用它装逼

你可以把这个项目包装成以下方向之一：

### 方向 A：课程设计 / MATLAB 大作业
标题可以写成：

> 基于卡尔曼滤波与协同控制的多无人机动态目标搜索跟踪系统设计

### 方向 B：简历项目
可以写成：

> Developed a MATLAB multi-agent search-and-track simulation system with dynamic target estimation, cooperative task allocation, obstacle avoidance, and real-time mission visualization.

### 方向 C：继续升级成论文味道
你可以加上：
- 目标丢失重捕获机制
- 观测概率模型
- 通信拓扑图
- 不同策略对比实验

---

## 推荐你先改的几个参数

在 `defaultMissionConfig.m` 里你可以直接改：

- `cfg.numAgents`：无人机数量
- `cfg.numTargets`：目标数量
- `cfg.totalTime`：总仿真时长
- `cfg.sensorRange`：探测半径
- `cfg.numObstacles`：障碍物数量
- `cfg.trackersPerTarget`：每个目标最多分配几架无人机
- `cfg.makeVideo`：是否导出视频

---

## 你还能继续问我什么

你下一步可以直接让我继续帮你做：

- “把它升级成 GUI 版 MATLAB App”
- “把这个项目改成更像毕业设计的格式”
- “给我加论文式摘要、关键词、系统框图”
- “给我写课程答辩讲稿”
- “给我再做一个图像处理/控制/优化类 MATLAB 项目”

