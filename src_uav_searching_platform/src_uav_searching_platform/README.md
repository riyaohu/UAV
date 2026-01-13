# UAV Search Algorithm Simulator

无人机搜索算法仿真平台 - 一个可视化的二维无人机目标搜索仿真系统

## 功能特性

- 可视化的二维地图环境
- 无人机实时搜索仿真
- 目标检测机制（基于距离阈值）
- 可扩展的搜索算法框架
- 实时统计信息显示
- 轨迹追踪和可视化

## 项目结构

```
src_uav_searching_platform/
├── img/                          # 图片资源
│   ├── uav.png                   # 无人机图片
│   ├── target.png                # 目标图片
│   └── map_1.png                 # 地图背景
├── algorithms/                   # 搜索算法模块
│   ├── __init__.py
│   ├── base_algorithm.py         # 算法基类
│   └── random_search.py          # 随机搜索算法（baseline）
├── config.py                     # 配置参数
├── main.py                       # 主程序入口
├── simulator.py                  # 仿真器核心
├── uav.py                        # 无人机类
├── target.py                     # 目标类
├── map_manager.py                # 地图管理
└── requirements.txt              # 依赖包
```

## 安装依赖

```bash
pip install -r requirements.txt
```

或直接安装：
```bash
pip install pygame
```

## 运行程序

```bash
python main.py
```

## 操作说明

- **SPACE** - 暂停/继续
- **R** - 重置仿真
- **ESC** - 退出程序

## 核心功能说明

### 1. 无人机（UAV）
- 位置和朝向追踪
- 移动轨迹记录
- 检测半径范围可视化
- 统计数据收集（飞行距离、时间等）

### 2. 目标（Target）
- 固定位置设定
- 检测状态管理
- 被发现时高亮显示

### 3. 搜索算法

#### 随机搜索（Random Search - Baseline）
- 随机方向移动
- 边界碰撞反弹
- 平滑转向机制

#### 扩展算法（可添加）
所有算法都继承 `BaseAlgorithm` 基类，只需实现 `get_next_position()` 方法即可。

### 4. 可视化界面
- 实时地图显示
- 无人机和目标图标
- 检测范围圆圈
- 飞行轨迹线
- 信息面板（时间、距离、状态等）

## 参数配置

在 `config.py` 中可以调整以下参数：

- **窗口设置**: 窗口大小、帧率
- **无人机设置**: 速度、检测半径、起始位置
- **目标设置**: 位置、大小
- **算法参数**: 转向概率、最大转向角度
- **可视化选项**: 是否显示检测范围、轨迹等

## 添加新算法

1. 在 `algorithms/` 目录下创建新的算法文件
2. 继承 `BaseAlgorithm` 类
3. 实现 `get_next_position()` 方法
4. 在 `algorithms/__init__.py` 中导入
5. 在 `simulator.py` 中使用新算法

示例：
```python
from algorithms.base_algorithm import BaseAlgorithm

class MyNewAlgorithm(BaseAlgorithm):
    def __init__(self, map_width, map_height):
        super().__init__(map_width, map_height)
        self.name = "My New Algorithm"

    def get_next_position(self, current_x, current_y, current_angle, **kwargs):
        # 实现你的算法逻辑
        next_x = current_x + ...
        next_y = current_y + ...
        return next_x, next_y
```

## 性能指标

仿真器会自动记录以下性能指标：
- 搜索时间（帧数和秒数）
- 飞行距离（像素）
- 当前位置
- 目标是否找到

## 技术栈

- **Python 3.x**
- **Pygame** - 图形渲染和游戏循环

## 未来扩展方向

- [ ] 多目标搜索
- [ ] 多无人机协同搜索
- [ ] 更多搜索算法（螺旋搜索、网格搜索、智能算法等）
- [ ] 障碍物避障
- [ ] 动态目标
- [ ] 数据导出和分析
- [ ] 算法性能对比可视化

## 作者

本科生毕业设计 - 无人机搜索算法仿真平台研究

## 许可证

MIT License

## 版本功能
- **main.py中render=False/True决定是否有运行界面 ,mode="experiment/demo决定结束条件（自动/人为关闭）**
- **experiment_runner运行实现多次仿真**
- 