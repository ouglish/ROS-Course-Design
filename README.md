# ROS-Course-Design

本仓库包含 ROS 课程设计代码，当前主要内容为三轮差速小车的多点导航节点。

---

## multi_point_nav —— 多点导航节点

### 问题分析与解决方案

#### 问题1：原地旋转后 x、y 坐标产生偏移

**现象**
目标点的 x、y 坐标不变，仅角度（theta）发生变化时，小车完成原地旋转并停止后，实际的 x、y 坐标与目标值存在偏差。

**原因**
三轮差速小车在原地旋转时，左右驱动轮以相反方向旋转。由于以下因素，机器人重心会发生轻微位移：
- 两侧驱动轮直径存在细微加工误差
- 地面摩擦不均匀（地面凹凸、轮胎磨损差异）
- 车体重心并不严格位于两轮连线中点
- 里程计积累误差

**解决方案**
在 `multi_point_nav.py` 中，对"仅角度变化"的目标点进行特殊处理：

1. **识别旋转目标**：若相邻两个目标点的 x、y 差值均在 `position_tolerance` 以内，且 theta 差值超过 `angle_tolerance`，则判定为"仅旋转目标"。
2. **旋转后漂移检测**：旋转完成后，等待机器人完全停止，再从 AMCL 获取当前位姿，计算与目标 (x, y) 的实际距离。
3. **自动位置修正**：若漂移距离超过 `xy_correction_threshold`（默认 0.05 m），则自动向 move_base 发送一个精确位置修正目标，将小车驶回正确坐标，并保持目标朝向不变。

**参数调整**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `position_tolerance` | 0.1 m | 判断"仅旋转"的 x/y 差值容差 |
| `angle_tolerance` | 0.1 rad | 判断角度变化"显著"的阈值 |
| `xy_correction_threshold` | 0.05 m | 触发 x/y 漂移修正的阈值，可根据小车机械精度调整 |

---

#### 问题2：导航默认起点与建图时的起始位置/朝向不一致

**现象**
使用 SLAM（如 gmapping）建图后，重新启动导航时，AMCL 初始定位出现偏差，小车在地图上的显示位置和实际位置不符。

**原因**
- 建图时，机器人的起始位置被记录为地图坐标系原点（0, 0, 0）。
- 导航重启后，AMCL 默认以 (0, 0, 0) 作为初始估计值。
- 若机器人**实际放置位置**或**朝向**与建图起点不完全一致（例如机器人被移动过、朝向有偏差），或建图时并非从地图原点出发，AMCL 的粒子滤波就会从错误的位置开始，导致定位收敛缓慢甚至失败。

**解决方案**
在 `multi_point_nav.py` 启动时，自动向 AMCL 的 `/initialpose` 话题发布初始位姿：

1. 通过 ROS 参数（`initial_pose_x`、`initial_pose_y`、`initial_pose_theta`）配置建图时的起始坐标和朝向。
2. 节点启动后立即发布 `PoseWithCovarianceStamped` 消息，告知 AMCL 机器人的当前位置，使粒子滤波从正确的位置开始。
3. 等待 AMCL 收敛（约 2 秒）后再开始导航。

**使用方式**

如果建图时机器人从地图原点 (0, 0) 出发且朝向正东，保持默认参数即可（默认值均为 0）。

若建图起点不在地图原点，在 launch 文件中修改以下参数：
```xml
<arg name="initial_pose_x"     default="1.5"/>   <!-- 建图起始 x 坐标（米） -->
<arg name="initial_pose_y"     default="-0.8"/>  <!-- 建图起始 y 坐标（米） -->
<arg name="initial_pose_theta" default="1.5708"/><!-- 建图起始朝向（弧度），此处为正北 -->
```

或在命令行中直接传入：
```bash
roslaunch multi_point_nav multi_point_nav.launch \
    initial_pose_x:=1.5 initial_pose_y:=-0.8 initial_pose_theta:=1.5708
```

---

### 快速开始

#### 1. 编译

```bash
cd ~/catkin_ws
catkin_make --pkg multi_point_nav
source devel/setup.bash
```

#### 2. 修改目标点

编辑 `multi_point_nav/config/nav_points.yaml`，按需增删目标点：

```yaml
waypoints:
  - x: 1.0
    y: 0.0
    theta: 0.0       # 正东方向
  - x: 1.0
    y: 1.0
    theta: 1.5708    # 正北方向（π/2）
  - x: 1.0           # x/y 与上一点相同，只改变角度
    y: 1.0           # → 自动识别为"仅旋转目标"
    theta: 3.1416    # 正西方向（π）
```

#### 3. 启动导航

```bash
# 使用默认参数（建图起点为地图原点，朝向正东）
roslaunch multi_point_nav multi_point_nav.launch

# 指定建图起点（如果建图时起点不在地图原点）
roslaunch multi_point_nav multi_point_nav.launch \
    initial_pose_x:=1.5 initial_pose_y:=-0.8 initial_pose_theta:=1.5708
```

#### 4. 循环导航

```bash
roslaunch multi_point_nav multi_point_nav.launch loop_waypoints:=true
```

---

### 文件结构

```
multi_point_nav/
├── CMakeLists.txt
├── package.xml
├── README.md（本文件）
├── config/
│   └── nav_points.yaml        # 目标点配置
├── launch/
│   └── multi_point_nav.launch # 启动文件（含所有参数说明）
└── scripts/
    └── multi_point_nav.py     # 多点导航节点主程序
```