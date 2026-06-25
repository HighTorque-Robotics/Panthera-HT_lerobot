# Panthera LeRobot 集成教学文档

## 📚 目录
1. [概述](#概述)
2. [架构设计](#架构设计)
3. [PantheraFollower 详解](#pantherafollower-详解)
4. [PantheraLeader 详解](#pantheraLeader-详解)
5. [PantheraMotorsBus 详解](#pantheramotorsbus-详解)
6. [使用示例](#使用示例)
7. [关键概念解析](#关键概念解析)

---

## 概述

本文档详细介绍 Panthera 机械臂如何集成到 LeRobot 框架中，主要包含三个核心模块：

- **PantheraFollower**: 机器人执行端（Follower），负责执行动作指令
- **PantheraLeader**: 机器人遥操作端（Leader），负责采集人工示教数据
- **PantheraMotorsBus**: 底层电机总线，封装 Panthera SDK 的硬件接口

### LeRobot 框架继承关系

```
LeRobot 基类
├── Robot (lerobot.robots.robot.Robot)
│   └── PantheraFollower  ← 执行端实现
│
├── Teleoperator (lerobot.teleoperators.teleoperator.Teleoperator)
│   └── PantheraLeader  ← 遥操作端实现
│
└── MotorsBus (lerobot.motors.motors_bus.MotorsBus)
    └── PantheraMotorsBus  ← 电机通信层
```

---

## 架构设计

### 1. 数据流向

```
人工示教模式:
PantheraLeader (读取位置) → LeRobot Dataset (记录) → 训练模型

自主执行模式:
训练好的模型 (推理) → PantheraFollower (执行动作) → Panthera 机械臂
```

### 2. 文件位置

```
lerobot_robot_panthera/
├── src/lerobot_robot_panthera/
│   ├── panthera_follower.py      # Follower 实现
│   ├── panthera_leader.py        # Leader 实现
│   └── motors/
│       └── panthera_motors_bus.py  # 电机总线实现
```

### 3. 依赖关系

- **Panthera SDK**: `hightorque_robot` + `Panthera_lib.Panthera`
- **LeRobot**: `lerobot.robots`, `lerobot.teleoperators`, `lerobot.motors`
- **其他**: `numpy`, `yaml`, `pinocchio` (动力学计算), `threading` (并发控制)

---

## PantheraFollower 详解

### 类继承关系

```python
class PantheraFollower(Robot):
    """
    文件位置: panthera_follower.py:105
    继承自: lerobot.robots.robot.Robot
    """
```

### 核心职责

1. **执行动作**: 接收策略模型输出的关节角度，发送到机械臂
2. **状态读取**: 实时读取关节位置和相机图像
3. **重力补偿**: 后台线程持续进行重力+摩擦补偿，使机械臂保持柔顺
4. **数据采集**: 支持与 LeRobot 数据集系统无缝对接

### 关键属性

| 属性 | 类型 | 说明 | 代码位置 |
|------|------|------|----------|
| `config` | `PantheraFollowerConfig` | 配置对象，包含相机配置和参数路径 | :114 |
| `bus` | `PantheraMotorsBus` | 电机总线实例 | :135 |
| `cameras` | `dict` | 相机字典，包含 wrist 和 top 相机 | :156 |
| `joint_names` | `list[str]` | 关节名称列表 (含夹爪) | :142-154 |
| `motor_map` | `dict` | 电机名到关节名的映射 | :213-221 |
| `_thread` | `Thread` | 重力补偿后台线程 | :244 |

### 核心方法解析

#### 1. `connect(calibrate=True)` - 连接设备

**代码位置**: panthera_follower.py:201

```python
def connect(self, calibrate=True):
    """
    连接步骤:
    1. 检查连接状态，避免重复连接
    2. 连接电机总线 (self.bus.connect())
    3. 建立电机到关节的映射关系
    4. 连接所有相机
    5. 启动重力补偿后台线程
    """
```

**关键代码片段**:
```python
# 电机-关节映射 (panthera_follower.py:216-221)
for i, name in enumerate(self.joint_names):
    if i < len(self.bus.motor_names):
        m_name = self.bus.motor_names[i]
        self.motor_map_inv[name] = m_name  # joint -> motor
        self.motor_map[m_name] = name      # motor -> joint
```

#### 2. `get_observation()` - 获取观测数据

**代码位置**: panthera_follower.py:277

```python
def get_observation(self):
    """
    返回格式:
    {
        "joint1.pos": 0.123,       # 关节位置 (rad)
        "joint2.pos": -0.456,
        ...
        "gripper.pos": 1.6,
        "wrist": np.array(...),    # RGB 图像 (H,W,3)
        "top": np.array(...),
        # 可选深度图:
        "wrist_depth": np.array(...),  # (H,W,1)
    }
    """
```

**实现细节**:
- 通过 `self.bus.sync_read("Present_Position")` 批量读取电机位置
- 使用 `motor_map` 将电机名转换为关节名
- 相机使用 `async_read()` 异步读取，带超时重试机制 (panthera_follower.py:291-305)

#### 3. `send_action(action)` - 执行动作指令

**代码位置**: panthera_follower.py:323

```python
def send_action(self, action):
    """
    输入格式:
    action = {
        "joint1.pos": 0.5,
        "joint2.pos": -0.3,
        ...
        "gripper.pos": 1.0
    }

    控制策略:
    - PD控制 + 重力前馈补偿
    - Kp/Kd 针对不同关节优化 (panthera_follower.py:367-380)
    - 关节力矩限制，防止过载 (panthera_follower.py:346-353)
    """
```

**关键代码分析**:
```python
# 重力补偿计算 (panthera_follower.py:343)
tau_gravity = robot.get_Gravity()  # 返回 6 个关节的重力力矩

# PD增益配置 (panthera_follower.py:367-368)
kp_defaults = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0, 4.0]  # 位置增益
kd_defaults = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1, 0.4]       # 速度增益
# 注意: 索引 6 是夹爪的增益 (kp=4.0, kd=0.4)

# 底层控制指令 (panthera_follower.py:400-406)
robot.Motors[i].pos_vel_tqe_kp_kd(
    target_pos[i],   # 目标位置
    target_vel[i],   # 目标速度 (这里为0)
    target_tqe[i],   # 前馈力矩 (重力补偿)
    kp[i],           # 位置增益
    kd[i]            # 速度增益
)
```

#### 4. `_control_loop()` - 重力补偿线程

**代码位置**: panthera_follower.py:415

```python
def _control_loop(self):
    """
    后台线程功能:
    1. 200Hz 频率运行 (5ms 循环)
    2. 当 0.5s 无外部指令时，自动应用重力+摩擦补偿
    3. 使机械臂保持被动柔顺状态 (可手动拖动)

    补偿公式:
    τ_total = τ_gravity + τ_friction

    其中:
    - τ_gravity: 重力补偿 (基于 Pinocchio 动力学)
    - τ_friction: 摩擦补偿 (库仑摩擦 + 粘性摩擦)
    """
```

**为什么需要后台线程?**
- **问题**: 机械臂在不动作时会下垂 (重力作用)
- **解决**: 持续施加重力补偿力矩，抵消重力
- **效果**: 机械臂可以悬停在任意位置，手动拖动时无明显阻力

### 配置类 PantheraFollowerConfig

**代码位置**: panthera_follower.py:98

```python
@RobotConfig.register_subclass("panthera_follower")
@dataclass
class PantheraFollowerConfig(RobotConfig):
    param_path: str = "robot_param/Follower.yaml"  # Panthera配置文件
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras_factory)
```

**相机配置** (panthera_follower.py:29-95):
```python
def default_cameras_factory():
    """
    自动检测 RealSense 相机:
    - D405 (352122273105) → wrist (手腕相机)
    - D435 (948122071707) → top   (顶部相机)

    配置:
    - 分辨率: 640x480
    - 帧率: 30 FPS
    - 色彩模式: RGB
    - 深度: 默认关闭 (可启用)
    """
```

### 夹爪处理逻辑

**预先添加夹爪** (panthera_follower.py:150-154):
```python
if len(self.joint_names) == 6:  # 如果只有 6 个关节
    self.joint_names.append("gripper")
    logger.info("预先添加夹爪到 joint_names")
```

**为什么要预先添加?**
- LeRobot 的数据集特征在初始化时就确定
- 如果 `action_features` 不包含 `gripper.pos`，后续无法记录夹爪数据
- 必须在调用 `action_features` 属性之前添加

---

## PantheraLeader 详解

### 类继承关系

```python
class PantheraLeader(Teleoperator):
    """
    文件位置: panthera_leader.py:40
    继承自: lerobot.teleoperators.teleoperator.Teleoperator
    """
```

### 核心职责

1. **采集示教数据**: 实时读取 Leader 端关节位置，提供给数据记录系统
2. **被动柔顺控制**: 施加重力+摩擦补偿，使机械臂易于手动拖动
3. **夹爪状态同步**: 保持夹爪在固定位置 (1.6 rad)

### 关键属性

| 属性 | 类型 | 说明 | 代码位置 |
|------|------|------|----------|
| `bus` | `PantheraMotorsBus` | 电机总线实例 | :87 |
| `joint_names` | `list[str]` | 关节名称 (含夹爪) | :91-108 |
| `_thread` | `Thread` | 补偿控制后台线程 | :115 |
| `_latest_action` | `dict` | 最新的关节位置数据 | :117 |
| `fc`, `fv` | `np.array` | 摩擦补偿系数 | :122-123 |

### 核心方法解析

#### 1. `connect(calibrate=True)` - 连接设备

**代码位置**: panthera_leader.py:177

```python
def connect(self, calibrate=True):
    """
    连接步骤:
    1. 连接电机总线
    2. 建立电机-关节映射
    3. 检测并添加夹爪映射 (如果有第7个电机)
    4. 启动重力补偿后台线程
    """
```

**夹爪检测逻辑** (panthera_leader.py:216-225):
```python
if len(self.bus.htr_motors) > 6:  # 电机数量超过6个
    gripper_name = "gripper"
    gripper_idx = 6  # 第7个电机 (索引为6)

    m_name = self.bus.motor_names[gripper_idx]
    self.motor_map_inv[gripper_name] = m_name
    self.motor_map[m_name] = gripper_name
```

#### 2. `get_action()` - 获取示教动作

**代码位置**: panthera_leader.py:273

```python
def get_action(self):
    """
    返回格式:
    {
        "joint1.pos": 0.123,
        "joint2.pos": -0.456,
        ...
        "gripper.pos": 1.6
    }

    数据来源:
    - 优先返回后台线程更新的 _latest_action
    - 线程未就绪时同步读取电机位置
    """
```

**线程安全设计**:
```python
with self._action_lock:  # 使用锁保护共享数据
    if not self._latest_action:
        # 回退方案: 同步读取
        motor_vals = self.bus.sync_read("Present_Position")
        ...
    return self._latest_action.copy()  # 返回副本，避免外部修改
```

#### 3. `_control_loop()` - 补偿控制线程

**代码位置**: panthera_leader.py:323

```python
def _control_loop(self):
    """
    200Hz 控制循环，执行:
    1. 读取关节位置和速度
    2. 更新 _latest_action (供 get_action 使用)
    3. 计算重力+摩擦补偿力矩
    4. 发送纯力矩控制指令 (Kp=0, Kd=0)
    5. 特殊处理夹爪: 保持在 1.6 位置
    """
```

**关键代码分析**:

**力矩补偿计算** (panthera_leader.py:379-391):
```python
# 重力补偿 (基于 Pinocchio RNEA 算法)
tau_gravity = robot.get_Gravity()

# 摩擦补偿 (Coulomb + Viscous)
tau_friction = robot.get_friction_compensation(
    vel=vel,                      # 当前速度
    Fc=self.fc,                   # 库仑摩擦系数
    Fv=self.fv,                   # 粘性摩擦系数
    vel_threshold=self.vel_threshold  # 速度阈值 (0.02 rad/s)
)

tau_total = tau_gravity + tau_friction
```

**力矩限制** (panthera_leader.py:394-404):
```python
tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

# 处理夹爪力矩限制
if len(tau_total) > len(tau_limit):
    tau_limit = np.append(tau_limit, [5.0] * (len(tau_total) - len(tau_limit)))

tau_total = np.clip(tau_total, -tau_limit, tau_limit)
```

**控制指令发送** (panthera_leader.py:412-419):
```python
# 前6个关节: 纯力矩控制
for i in range(6):
    robot.Motors[i].pos_vel_tqe_kp_kd(
        0.0,           # 位置命令 (忽略)
        0.0,           # 速度命令 (忽略)
        tau_total[i],  # 补偿力矩
        0.0,           # Kp = 0
        0.0            # Kd = 0
    )

# 夹爪: 位置控制保持在 1.6
robot.Motors[6].pos_vel_tqe_kp_kd(1.6, 0.0, 0.0, 0.2, 0.02)

robot.motor_send_cmd()  # 统一发送
```

### 配置类 PantheraLeaderConfig

**代码位置**: panthera_leader.py:21

```python
@TeleoperatorConfig.register_subclass("panthera_leader")
@dataclass
class PantheraLeaderConfig(TeleoperatorConfig):
    param_path: str = "robot_param/Leader.yaml"

    # 摩擦补偿参数 (从 SDK 示例调优)
    fc: tuple = (0.20, 0.15, 0.15, 0.15, 0.04, 0.04)  # 库仑摩擦
    fv: tuple = (0.06, 0.06, 0.06, 0.03, 0.02, 0.02)  # 粘性摩擦
    vel_threshold: float = 0.02  # 速度阈值
```

**摩擦补偿原理**:
```
τ_friction = Fc * sign(v) + Fv * v  (当 |v| > vel_threshold)
τ_friction = 0                      (当 |v| ≤ vel_threshold)

其中:
- Fc: 库仑摩擦系数 (静态摩擦)
- Fv: 粘性摩擦系数 (动态摩擦)
- v: 关节速度
- vel_threshold: 死区阈值，避免抖动
```

---

## PantheraMotorsBus 详解

### 类继承关系

```python
class PantheraMotorsBus(MotorsBus):
    """
    文件位置: panthera_motors_bus.py:49
    继承自: lerobot.motors.motors_bus.MotorsBus
    """
```

### 核心职责

1. **适配器模式**: 将 Panthera SDK 的接口适配为 LeRobot 的 MotorsBus 接口
2. **电机管理**: 管理多个电机的连接、读写、同步通信
3. **抽象硬件细节**: 屏蔽底层通信协议，提供统一接口

### 关键属性

| 属性 | 类型 | 说明 | 代码位置 |
|------|------|------|----------|
| `robot` | `Panthera` | Panthera SDK 机器人实例 | :59 |
| `htr_motors` | `list` | 底层 hightorque_robot 电机对象列表 | :60 |
| `motor_names` | `list[str]` | 电机名称 (motor_0, motor_1, ...) | :61 |
| `target_positions` | `np.array` | 目标位置缓冲区 | :65 |
| `_is_connected` | `bool` | 连接状态标志 | :62 |

### 核心方法解析

#### 1. `connect()` - 连接电机总线

**代码位置**: panthera_motors_bus.py:77

```python
def connect(self):
    """
    连接步骤:
    1. 检查 hightorque_robot 模块是否导入
    2. 初始化 Panthera 实例 (自动解析配置文件)
    3. 获取电机列表
    4. 初始化目标位置/速度/力矩缓冲区
    5. 发送初始读取指令稳定连接
    """
```

**关键代码**:
```python
# Panthera 类会自动处理配置路径解析
self.robot = Panthera(self.config_path)

# 获取电机对象
self.htr_motors = self.robot.get_motors()

# 命名电机
self.motor_names = [f"motor_{i}" for i in range(len(self.htr_motors))]

# 初始化缓冲区
num_motors = len(self.htr_motors)
self.target_positions = np.zeros(num_motors)
```

#### 2. `sync_read(data_name)` - 批量读取电机状态

**代码位置**: panthera_motors_bus.py:160

```python
def sync_read(self, data_name):
    """
    支持的数据类型:
    - "Present_Position": 当前位置 (rad)
    - "Present_Velocity": 当前速度 (rad/s)
    - "Present_Torque":   当前力矩 (Nm)

    返回格式:
    {
        "motor_0": 0.123,
        "motor_1": -0.456,
        ...
    }
    """
```

**实现细节**:
```python
# 发送读取指令
self.robot.send_get_motor_state_cmd()

# 遍历所有电机获取状态
for i, motor in enumerate(self.htr_motors):
    state = motor.get_current_motor_state()
    if data_name == "Present_Position":
        val = state.position
    elif data_name == "Present_Velocity":
        val = state.velocity
    # ...
    values[self.motor_names[i]] = val

return values
```

#### 3. `sync_write(data_name, values)` - 批量写入电机指令

**代码位置**: panthera_motors_bus.py:181

```python
def sync_write(self, data_name, values):
    """
    输入格式:
    values = {
        "motor_0": 0.5,
        "motor_1": -0.3,
        ...
    }

    支持的数据类型:
    - "Goal_Position": 目标位置
    - "Goal_Velocity": 目标速度
    - "Goal_Torque":   目标力矩
    """
```

**实现逻辑**:
```python
# 1. 更新缓冲区
for name, val in values.items():
    idx = self.motor_names.index(name)
    if data_name == "Goal_Position":
        self.target_positions[idx] = val

# 2. 发送指令到电机
for i in range(len(self.htr_motors)):
    p = self.target_positions[i]
    v = self.target_velocities[i]
    t = self.target_torques[i]

    self.htr_motors[i].pos_vel_MAXtqe(p, v, t)

# 3. 统一发送
self.robot.motor_send_cmd()
```

### 抽象方法实现

LeRobot 的 `MotorsBus` 基类定义了多个抽象方法，`PantheraMotorsBus` 根据需要覆写:

| 方法 | 实现状态 | 说明 |
|------|---------|------|
| `_assert_protocol_is_compatible` | 空实现 | Panthera SDK 内部处理协议 |
| `disable_torque` / `enable_torque` | 空实现 | SDK 自动管理 |
| `is_calibrated` | 返回 `True` | Panthera 使用绝对编码器 |
| `read_calibration` / `write_calibration` | 空实现 | 无需用户校准 |
| `broadcast_ping` | 返回伪数据 | 兼容性接口 |

---

## 使用示例

### 示例 1: 使用 PantheraFollower 执行动作

```python
from lerobot_robot_panthera.panthera_follower import PantheraFollower, PantheraFollowerConfig

# 1. 创建配置
config = PantheraFollowerConfig(
    param_path="robot_param/Follower.yaml"
)

# 2. 初始化机器人
robot = PantheraFollower(config)

# 3. 连接设备
robot.connect()

# 4. 获取观测
obs = robot.get_observation()
print("关节位置:", {k: v for k, v in obs.items() if k.endswith('.pos')})
print("相机:", obs['wrist'].shape, obs['top'].shape)

# 5. 发送动作
action = {
    "joint1.pos": 0.5,
    "joint2.pos": -0.3,
    "joint3.pos": 0.0,
    "joint4.pos": 0.2,
    "joint5.pos": 0.0,
    "joint6.pos": 0.0,
    "gripper.pos": 1.0
}
robot.send_action(action)

# 6. 断开连接
robot.disconnect()
```

### 示例 2: 使用 PantheraLeader 采集示教数据

```python
from lerobot_robot_panthera.panthera_leader import PantheraLeader, PantheraLeaderConfig

# 1. 创建配置
config = PantheraLeaderConfig(
    param_path="robot_param/Leader.yaml",
    fc=(0.20, 0.15, 0.15, 0.15, 0.04, 0.04),  # 摩擦补偿参数
    fv=(0.06, 0.06, 0.06, 0.03, 0.02, 0.02)
)

# 2. 初始化遥操作器
leader = PantheraLeader(config)

# 3. 连接设备
leader.connect()

# 4. 读取示教动作
action = leader.get_action()
print("示教动作:", action)

# 5. 断开连接
leader.disconnect()
```

### 示例 3: 完整的数据采集流程 (使用 LeRobot CLI)

```bash
# 1. 记录一个 episode
python lerobot/scripts/control_robot.py record \
    --robot-path lerobot_robot_panthera.panthera_follower \
    --robot-overrides param_path=robot_param/Follower.yaml \
    --teleop-path lerobot_robot_panthera.panthera_leader \
    --teleop-overrides param_path=robot_param/Leader.yaml \
    --fps 30 \
    --repo-id your_username/panthera_demo \
    --tags tutorial panthera \
    --warmup-time-s 5 \
    --episode-time-s 30 \
    --reset-time-s 10 \
    --num-episodes 50

# 2. 可视化数据
python lerobot/scripts/visualize_dataset.py \
    --repo-id your_username/panthera_demo \
    --episode-index 0

# 3. 训练策略
python lerobot/scripts/train.py \
    --dataset-repo-id your_username/panthera_demo \
    --policy-name act \
    --output-dir outputs/panthera_act
```

---

## 关键概念解析

### 1. 重力补偿 (Gravity Compensation)

**问题**: 机械臂在不施加控制力时会因为重力下垂

**解决方案**: 实时计算重力力矩并施加反向力矩

**实现**:
```python
# 使用 Pinocchio 动力学库计算
tau_gravity = robot.get_Gravity()  # 基于 RNEA 算法
robot.Motors[i].pos_vel_tqe_kp_kd(0, 0, tau_gravity[i], 0, 0)
```

**代码位置**:
- Follower: panthera_follower.py:441
- Leader: panthera_leader.py:380

### 2. 摩擦补偿 (Friction Compensation)

**摩擦模型**:
```
τ_friction = Fc * sign(v) + Fv * v
```

**参数含义**:
- `Fc` (Coulomb): 静态摩擦系数，与速度方向相关
- `Fv` (Viscous): 动态摩擦系数，与速度大小成正比
- `vel_threshold`: 速度死区，避免低速时符号函数抖动

**调参建议**:
1. 先调 `Fc`: 使机械臂在低速移动时无明显卡顿
2. 再调 `Fv`: 使高速移动时阻力适中
3. 调整 `vel_threshold`: 消除静止时的抖动

**代码位置**: panthera_leader.py:383-388

### 3. PD 控制 (Proportional-Derivative Control)

**控制公式**:
```
τ = Kp * (θ_target - θ_current) + Kd * (ω_target - ω_current) + τ_feedforward
```

**参数调优**:
- **Kp 增大**: 响应更快，但可能震荡
- **Kd 增大**: 阻尼更强，减少震荡，但会降低响应速度
- **前馈力矩**: 提高跟踪精度，减少稳态误差

**Follower 中的增益配置**:
```python
# panthera_follower.py:367-368
kp_defaults = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0, 4.0]
kd_defaults = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1, 0.4]

# 解释:
# - 关节 1 (基座): Kp=10, 需要较小增益避免底座晃动
# - 关节 2-3 (大臂): Kp=21, 最高增益，承载重力大
# - 关节 4-5 (小臂): Kp=16/13, 中等增益
# - 关节 6 (末端): Kp=1, 低增益，精细控制
# - 夹爪: Kp=4, 中等增益
```

### 4. 线程安全 (Thread Safety)

**问题**: Leader 的后台线程和主线程同时访问 `_latest_action`

**解决方案**: 使用 `threading.Lock` 互斥锁

```python
# panthera_leader.py:291
with self._action_lock:  # 获取锁
    if not self._latest_action:
        # 同步读取
        ...
    return self._latest_action.copy()  # 释放锁
```

**注意事项**:
- 锁内的代码要尽量简短，避免阻塞
- 返回副本 (`.copy()`) 避免外部修改影响内部数据

### 5. 特征定义 (Features)

**观测特征** (`observation_features`):
```python
# panthera_follower.py:190
{
    "joint1.pos": float,
    "joint2.pos": float,
    ...
    "gripper.pos": float,
    "wrist": (480, 640, 3),  # RGB image
    "top": (480, 640, 3)
}
```

**动作特征** (`action_features`):
```python
# panthera_follower.py:194
{
    "joint1.pos": float,
    "joint2.pos": float,
    ...
    "gripper.pos": float
}
```

**为什么要定义特征?**
- LeRobot 数据集需要知道每个数据的维度和类型
- 策略模型需要根据特征定义构建输入/输出层
- 特征不匹配会导致训练或推理失败

### 6. 夹爪处理策略

**Follower 夹爪控制** (panthera_follower.py:367):
```python
# 夹爪使用较小的 PD 增益
kp[6] = 4.0  # 位置增益
kd[6] = 0.4  # 速度增益
```

**Leader 夹爪控制** (panthera_leader.py:416):
```python
# 夹爪保持在固定位置 1.6 rad (半开状态)
robot.Motors[6].pos_vel_tqe_kp_kd(1.6, 0.0, 0.0, 0.2, 0.02)
```

**为什么 Leader 夹爪固定?**
- 简化示教操作，用户不需要手动调整夹爪
- 数据采集时可以通过手动开关来触发夹爪动作
- 也可以修改为跟随手动拖动 (需要添加力传感器)

### 7. 相机配置自动检测

**实现逻辑** (panthera_follower.py:36-70):
```python
available_cameras = RealSenseCamera.find_cameras()  # 检测所有相机

camera_map = {
    "352122273105": "wrist",  # D405 序列号
    "948122071707": "top"     # D435 序列号
}

for cam_info in available_cameras:
    serial = cam_info["id"]
    if serial in camera_map:
        key = camera_map[serial]
        cameras[key] = RealSenseCameraConfig(serial_number_or_name=serial, ...)
```

**自定义相机配置**:
```python
config = PantheraFollowerConfig(
    cameras={
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="your_serial",
            fps=60,  # 提高帧率
            width=1280,
            height=720,
            use_depth=True  # 启用深度
        )
    }
)
```

---

## 总结

### 设计亮点

1. **清晰的继承架构**: 遵循 LeRobot 框架设计规范
2. **后台线程设计**: 实现高频重力补偿，不阻塞主流程
3. **线程安全机制**: 使用锁保护共享数据
4. **灵活的配置系统**: 支持 YAML 配置和代码覆盖
5. **自动设备检测**: 相机自动识别，减少手动配置

### 注意事项

1. **配置文件路径**: 确保 `param_path` 正确指向 Panthera YAML 配置
2. **摩擦补偿参数**: 需要根据实际机械臂调优 `fc` 和 `fv`
3. **力矩限制**: 防止过载，保护机械臂安全
4. **线程清理**: `disconnect()` 时必须停止后台线程
5. **夹爪预添加**: 确保在 `action_features` 调用前添加

### 扩展建议

1. **力反馈**: 在 Leader 的 `feedback_features` 中添加力矩反馈
2. **夹爪动态控制**: Leader 端读取夹爪传感器，动态调整位置
3. **多机器人支持**: 扩展 MotorsBus 支持多个 Panthera 实例
4. **深度数据**: 启用 RealSense 深度功能，提高策略性能
5. **参数自动调优**: 实现摩擦补偿参数的在线学习

---

**文档版本**: 1.0
**最后更新**: 2026-01-26

