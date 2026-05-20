# 无人机搜索算法仿真平台（UAV Search Algorithm Simulator）

本科毕业设计项目：用于二维环境下无人机目标搜索算法的仿真、可视化与对照实验评估。项目构建了统一的仿真平台，集成了栅格地图环境、概率观测模型与信念图系统，并实现了多种搜索策略算法（随机搜索、覆盖式搜索、前沿搜索、信息增益搜索及基于模板匹配的视觉引导搜索），用于系统比较不同算法在复杂环境中的性能表现。

在GridMap实验中，环境为包含障碍物的二维栅格地图，地图中存在多个目标点，障碍物分布和起点采样由随机种子控制，目标位置采用配置文件中的固定多目标设置；视觉实验中，目标位置由随机种子生成。无人机需要在存在障碍约束的情况下完成搜索任务。

在视觉搜索实验中，环境不包含障碍物，目标为单一目标，主要用于评估模板匹配引导搜索与随机搜索在视觉感知驱动下的性能差异。

两类实验分别从复杂环境搜索能力与视觉引导策略效果两个角度，对算法性能进行了综合评估。

---

## 1. 当前已实现能力

### 1.1 基础仿真平台

- 二维地图可视化渲染（地图、无人机、目标、轨迹、探测半径）
- 多目标搜索流程（基于探测半径 + 概率漏检模型判定是否发现目标）
- 实时状态显示（帧数、时间、距离、位置、模式、已发现目标数）
- 自动终止条件（帧数上限 / 发现所有目标）

### 1.2 基线算法 A：随机搜索（Random Search）

- 位置：`algorithms/random_search.py`
- 特点：随机平滑转向、边界反弹，作为对照基线

### 1.3 基线算法 B：模板匹配引导搜索（Template Matching + 巡航）

- 位置：`baseline_template_matching_cruise/`
- 特点：
  - 多尺度模板匹配（OpenCV `TM_CCOEFF_NORMED`）
  - 三模式切换：`巡航模式` / `候选验证` / `视觉引导`
  - 蛇形巡航航点 + 边界切线处理
  - 指数平滑分数 + 转向限幅
  - 独立模块、独立配置、独立结果目录

### 1.4 基线算法 C：覆盖式搜索（Coverage Search）

- 位置：`algorithms/coverage_search.py`
- 特点：蛇形扫描策略，按固定步长扫描整张地图；支持障碍物绕行

### 1.5 基线算法 D：前沿搜索（Frontier Search）

- 位置：`algorithms/frontier_search.py`
- 特点：
  - 依赖信念图（Belief Grid）驱动决策
  - 持续朝向信息密度最高的未探索边界（Frontier）移动
  - 兼顾距离代价与信息增益

### 1.6 基线算法 E：信息增益搜索（Information Gain Search）

- 位置：`algorithms/information_gain.py`
- 特点：
  - 单步前瞻（One-step Lookahead）评估多方向信息增益
  - 有效覆盖比例修正（边缘/角落区域自动打折）
  - 方向惯性 + 转向惩罚 + 卡死检测与逃脱
  - 依赖信念图

### 1.7 栅格地图与障碍物环境（Grid Map）

- 位置：`environment/grid_map.py`
- 支持可配置的障碍物密度、边界封闭、随机空闲起点生成

### 1.8 概率观测模型（Observation Model）

- 位置：`observation/observation_model.py`
- 支持漏检概率（`P_FALSE_NEGATIVE`）、误检概率（`P_FALSE_POSITIVE`）、距离噪声

### 1.9 信念图系统（Belief Grid）

- 位置：`belief/grid_belief.py`
- 基于贝叶斯更新维护每个栅格的目标存在概率，供 Frontier / 信息增益算法使用
- 支持可视化叠加（透明度可配置）

### 1.10 对照实验与量化统计

- 入口：`python -m src_uav_searching_platform.evaluation.experiment_runner`
- 可统一对比所有算法（GridMap 实验 + 视觉实验）
- 自动输出：
  - 逐帧日志（按算法 / 回合归档）
  - 回合级结果汇总
  - 算法级统计（成功率、平均成功帧/时间、平均结束帧/距离）
  - 对照结论文本（时间改进率、成功率差值）
  - 可视化图表（4张：核心指标 / 回合速度 / 累计成功率 / 帧差）

---

## 2. 项目结构

```text
src_uav_searching_platform/
├── img/
│   ├── map_1.png
│   ├── uav.png
│   └── target.png
├── algorithms/
│   ├── base_algorithm.py
│   ├── random_search.py
│   ├── coverage_search.py          # 覆盖式蛇形扫描
│   ├── frontier_search.py          # 前沿搜索（依赖 belief）
│   ├── information_gain.py         # 信息增益（依赖 belief）
│   └── __init__.py
├── baseline_template_matching_cruise/
│   ├── __init__.py
│   ├── config_template_matching.py
│   ├── template_matching_cruise.py
│   ├── simulator_template_matching_cruise.py
│   └── requirements.txt
├── belief/
│   ├── grid_belief.py              # 信念栅格（贝叶斯更新）
│   └── __init__.py
├── environment/
│   └── grid_map.py                 # 栅格地图 + 障碍物
├── evaluation/
│   ├── experiment_runner.py        # 对照实验主入口
│   ├── metrics.py                  # 指标计算
│   └── plot_results.py             # 可视化图表生成
├── observation/
│   ├── observation_model.py        # 概率观测模型
│   └── __init__.py
├── utils/
│   └── seed.py                     # 随机种子工具
├── config.py                       # 全局配置
├── simulator.py                    # 核心仿真器
├── map_manager.py
├── target.py
├── uav.py
├── main.py                         # 可视化 demo 入口
├── requirements.txt
└── __init__.py
```

---

## 3. 环境依赖

### 3.1 安装全部依赖（推荐）

```bash
pip install -r requirements.txt
```

### 3.2 仅需模板匹配基线时

```bash
pip install -r baseline_template_matching_cruise/requirements.txt
```

---

## 4. 运行方式

> **注意**：本项目以 Python 包形式组织，所有命令需在 `src_uav_searching_platform/` **的上一级目录**下执行（即包含 `src_uav_searching_platform/` 文件夹的那一层）。

### 4.1 运行可视化 Demo（自定义算法运行）

除了通过 `main.py` 运行默认配置外，也可以直接在代码中构造 `Simulator` 实例，自定义运行不同算法：

```bash
python -m src_uav_searching_platform.main
```

main.py修改simulator = Simulator(render=True, mode="demo", use_grid_map=False, algorithm_name="template_matching")的参数即可。

| 参数               | 取值                        | 说明                                   |
| ---------------- | ------------------------- | ------------------------------------ |
| `render`         | True / False              | 是否开启可视化（实验时一般设为 False 提速）            |
| `mode`           | `"demo"` / `"experiment"` | demo：交互运行；experiment：自动运行并退出         |
| `use_grid_map`   | True / False              | True：栅格地图（用于四算法对比）False：图片地图（用于视觉实验） |
| `algorithm_name` | 字符串                       | 指定运行算法                               |

#### 可选算法列表

**GridMap（use_grid_map=True）：**

- "random"
- "coverage"
- "frontier"
- "information_gain"

**视觉实验（use_grid_map=False）：**

- "random"
- "template_matching"

### 4.2 运行完整对照实验

同时评估 GridMap 上的四种算法（随机 / 覆盖 / 前沿 / 信息增益）以及视觉实验（随机搜索 vs 模板匹配）：

```bash
python -m src_uav_searching_platform.evaluation.experiment_runner
```

可在 `experiment_runner.py` 顶部修改以下参数：

| 参数             | 默认值    | 说明                 |
| -------------- | ------ | ------------------ |
| `num_runs`     | `5`    | 每种算法的实验回合数         |
| `base_seed`    | `200`  | 基础随机种子（GridMap 实验） |
| `use_grid_map` | `True` | 是否启用栅格地图           |

视觉实验固定使用 `base_seed=20260408`，回合数 `5`（与老师基线对齐）。

---

## 5. 输出结果说明

所有结果保存在 `results/<时间戳>/` 目录下。

### 5.1 目录结构示例

```text
results/20260428_120000/
├── 场景列表.csv
├── 回合级结果.csv
├── 算法对照统计.csv
├── 对照结论.txt
├── random/
│   ├── run_results.csv
│   └── 回合_000_逐帧日志.csv  ...
├── coverage/
├── frontier/
├── information_gain/
├── random_search/              # 视觉实验（老师模式）
│   └── 回合_001_逐帧日志.csv  ...
├── template_matching_cruise/   # 视觉实验（老师模式）
│   └── 回合_001_逐帧日志.csv  ...
└── 可视化图表/
    ├── GridMap_图1_核心指标对照.png
    ├── GridMap_图2_回合发现速度对照.png
    ├── GridMap_图3_累计成功率曲线.png
    ├── GridMap_图4_相对随机帧差.png
    ├── Vision_图1_核心指标对照.png
    ├── Vision_图2_回合发现速度对照.png
    ├── Vision_图3_累计成功率曲线.png
    ├── Vision_图4_相对随机帧差.png
    └── 可视化说明.txt
```

### 5.2 关键 CSV 字段说明

**逐帧日志**（每种算法每回合一个文件）：

```
时间帧, x坐标, y坐标, 累计距离, 已发现目标数, 是否完成
```

视觉实验逐帧日志还包含（与老师格式对齐）：

```
时间帧, 算法, 模式, 匹配分数, 平滑分数, x坐标, y坐标, 目标是否锁定, 是否发现目标, 累计飞行距离, 目标x, 目标y
```

**回合级结果**（`回合级结果.csv`）：

```
实验组, 算法, 回合, 随机种子, 是否成功, 已发现目标数, 总目标数, 发现率, 结束帧数, 结束距离, 停止原因, ...
```

**算法对照统计**（`算法对照统计.csv`）：

```
实验组, 算法, 总回合数, 成功率, 平均结束帧数, 平均结束距离, 平均成功帧数, 平均成功时间(秒)
```

---

## 6. 评估指标

### 6.1 通用指标（GridMap 实验 + 视觉实验）

- **成功率**：成功回合占总回合数的比例
- **平均成功帧数**：仅统计成功回合，发现目标时刻的平均帧号
- **平均成功时间（秒）**：平均成功帧数 ÷ FPS
- **平均结束帧数**：含失败回合在内的所有回合结束帧均值
- **平均结束距离（像素）**：含失败回合的累计飞行距离均值

### 6.2 视觉实验专项对比指标

- **平均成功时间相对改进（%）**：以随机搜索为基线，模板匹配的时间缩短比例
- **成功率差值（%）**：模板匹配成功率 − 随机搜索成功率

---

## 7. 键盘操作（可视化 Demo 模式）

| 按键      | 功能      |
| ------- | ------- |
| `SPACE` | 暂停 / 继续 |
| `R`     | 重置      |
| `ESC`   | 退出      |

---

## 8. 主要配置项（`config.py`）

| 配置项                     | 默认值          | 说明        |
| ----------------------- | ------------ | --------- |
| `WINDOW_WIDTH / HEIGHT` | `1200 / 800` | 窗口尺寸（像素）  |
| `FPS`                   | `60`         | 仿真帧率      |
| `UAV_SPEED`             | `3.0`        | 无人机每帧移动像素 |
| `UAV_DETECTION_RADIUS`  | `80`         | 探测半径（像素）  |
| `MAX_FRAMES`            | `10000`      | 单回合最大帧数   |
| `NUM_TARGETS`           | `3`          | 多目标数量     |
| `USE_GRID_MAP`          | `True`       | 是否启用栅格地图  |
| `CELL_SIZE`             | `20`         | 栅格大小（像素）  |
| `OBSTACLE_DENSITY`      | `0.02`       | 障碍物密度     |
| `P_FALSE_NEGATIVE`      | `0.3`        | 漏检概率      |
| `P_FALSE_POSITIVE`      | `0.0`        | 误检概率      |
| `DEFAULT_ALGORITHM`     | `"random"`   | Demo 默认算法 |
| `DEFAULT_SEED`          | `200`        | Demo 固定种子 |

---

## 9. 说明

- 基线行为可对齐复现。
- 算法（Coverage / Frontier / InformationGain）均继承 `BaseAlgorithm`，通过 `algorithm_name` 参数在 `Simulator` 中动态加载，不破坏原有随机搜索主流程。
- 结果输出字段与对照实验格式兼容（特别是视觉实验的逐帧日志表头）。

---

## 附录 A：论文可直接引用的实验流程

> 以下内容可作为论文"实验设计与评价方法"小节初稿使用，按实际参数微调即可。

### A.1 实验设置

本研究在统一二维仿真平台上，对五种无人机目标搜索策略进行系统对照实验：

- **策略 1**：随机搜索（Random Search）—— 对照基线
- **策略 2**：覆盖式搜索（Coverage Search）—— 确定性蛇形扫描
- **策略 3**：前沿搜索（Frontier Search）—— 信念图驱动的边界探索
- **策略 4**：信息增益搜索（Information Gain Search）—— 单步前瞻最大化信息量
- **策略 5**：模板匹配引导搜索（Template Matching + 巡航）—— 视觉基线对比

实验分为两组：

**GridMap 实验**：策略 1–4 在配有障碍物的栅格地图上运行，每种算法独立运行 `num_runs` 回合，每回合共享相同随机种子生成的起点与目标位置，以保证对比公平性。

**视觉实验**：策略 1（随机搜索）与策略 5（模板匹配引导搜索）在真实地图图像场景下运行，两算法共享同一场景（相同起点、目标位置、随机种子），仅决策策略不同。

实验通过以下命令运行：

```bash
python -m src_uav_searching_platform.evaluation.experiment_runner
```

其中 GridMap 实验参数：`num_runs=5`，`base_seed=200`；视觉实验参数：`num_runs=5`，`base_seed=20260408`。

### A.2 指标定义

设总回合数为 $N$，第 $i$ 回合是否成功记为 $s_i \in \{0,1\}$，成功发现时刻（帧）记为 $f_i$，回合结束帧记为 $e_i$，结束累计飞行距离记为 $d_i$，仿真帧率为 $FPS$。

1. **成功率**：

$$SR = \frac{1}{N}\sum_{i=1}^{N} s_i$$

2. **平均成功帧数**（仅统计成功回合）：

$$\overline{F}_{succ} = \frac{1}{\sum s_i} \sum_{i:s_i=1} f_i$$

3. **平均成功时间（秒）**：

$$\overline{T}_{succ} = \frac{\overline{F}_{succ}}{FPS}$$

4. **平均结束帧数**（含失败回合）：

$$\overline{E} = \frac{1}{N}\sum_{i=1}^{N} e_i$$

5. **平均结束距离**（像素，含失败回合）：

$$\overline{D} = \frac{1}{N}\sum_{i=1}^{N} d_i$$

6. **平均成功时间相对改进率**（视觉实验，以随机搜索为基线）：

$$\mathrm{Gain}_{time} = \frac{\overline{T}_{random} - \overline{T}_{template}}{\overline{T}_{random}} \times 100\%$$

### A.3 统计方法

1. **回合级记录**：每回合分别记录各算法的是否成功、发现帧号、结束帧号、结束距离，对应 `回合级结果.csv`。
2. **算法级汇总**：基于回合级数据计算各项统计指标，对应 `算法对照统计.csv`。
3. **自动结论生成**：程序自动输出成功率差值与平均成功时间相对改进率，对应 `对照结论.txt`。
4. **可视化图表**：自动生成核心指标对照柱状图、逐回合速度折线图、累计成功率曲线、配对帧差图，共 8 张（GridMap 4 张 + 视觉实验 4 张）。

### A.4 结果解释建议

- 若覆盖式搜索在平均结束帧数上优于随机搜索，可说明系统性路径规划相对随机游走具有效率优势；
- 若前沿搜索或信息增益搜索在成功率上领先，可说明利用信念图引导决策有效减少了重复探索；
- 若模板匹配在视觉实验中成功率与发现时间均优于随机搜索，可说明视觉先验信息的引入切实加速了目标定位；
- 若各算法在障碍物场景下与空地场景下表现差异显著，需结合环境复杂度与算法容错机制进行分析。
