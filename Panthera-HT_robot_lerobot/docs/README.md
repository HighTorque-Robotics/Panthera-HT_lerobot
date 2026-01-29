# LeRobot Panthera-HT 使用文档

本文档包含 LeRobot Panthera-HT 机器人的完整使用指令、支持的算法以及常见问题解决方案。

---

## 目录

- [快速开始](#快速开始)
  - [数据采集](#数据采集)
  - [模型训练](#模型训练)
  - [模型评估](#模型评估)
  - [数据可视化](#数据可视化)
- [支持的训练算法](#支持的训练算法)
- [数据集结构说明](#数据集结构说明)
- [常见问题](#常见问题)
- [相机配置](#相机配置)

---

## 快速开始

> **重要提示：** 所有以下命令都需要在 `Panthera-HT_lerobot/lerobot/src/lerobot/scripts` 目录下运行。

```bash
# 进入脚本目录（假设您已在 Panthera-HT_LeRobot 项目根目录）
cd Panthera-HT_lerobot/lerobot/src/lerobot/scripts
```

### 数据采集

采集机器人遥操作数据集：

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --teleop.type=panthera_leader \
  --dataset.repo_id=local/panthera_gaoqing_test \
  --dataset.single_task="gaoqing_test" \
  --dataset.num_episodes=2 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --dataset.episode_time_s=10 \
  --play_sounds=false
```

**参数说明：**
- `--robot.type`: 机器人类型（follower 为执行端）
- `--teleop.type`: 遥操作类型（leader 为主控端）
- `--dataset.repo_id`: 数据集保存位置
- `--dataset.single_task`: 任务名称
- `--dataset.num_episodes`: 采集的回合数
- `--dataset.episode_time_s`: 每回合时长（秒）
- `--dataset.reset_time_s`: 录制间隙时长（默认60秒，可根据需要调整）

**录制行为说明：**

1. **录制期间**：从臂（follower）实时跟随主臂（leader）的动作
2. **录制完成后**（0.5秒后自动触发）：
   - 系统显示"进入位置保持模式，保持当前位置"
   - 从臂使用 PD 控制 + 重力补偿稳定保持在当前位置
3. **手动复位环境**：在录制间隙时间内，手动将机械臂和环境复位到初始状态
4. **开始下一次录制**：
   - 系统显示"退出位置保持模式"
   - 从臂恢复正常跟随主臂

**位置保持技术细节：**
- 使用经过验证的 PD 控制参数（Kp: [4.0, 10.0, 10.0, 2.0, 2.0, 1.0, 3.0], Kd: [0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.3]）
- 结合重力补偿和摩擦补偿，确保稳定性
- 控制频率：200Hz
- 自动捕获录制结束时的位置并保持

### 模型训练

#### 1. ACT (Action Chunking with Transformers)

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_gaoqing_test \
  --policy.type=act \
  --policy.n_obs_steps=1 \
  --policy.n_action_steps=50 \
  --policy.chunk_size=100 \
  --policy.push_to_hub=false \
  --output_dir=outputs/train/panthera_gaoqing_test \
  --wandb.enable=true \
  --wandb.project="lerobot panthera-HT" \
  --wandb.notes="在公司录制的数据进行训练" \
  --batch_size=4 \
  --steps=1000 \
  --save_freq=1000 \
  --log_freq=100
```

#### 2. Diffusion Policy

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_gaoqing_test5 \
  --policy.type=diffusion \
  --policy.n_obs_steps=2 \
  --policy.horizon=16 \
  --policy.n_action_steps=8 \
  --output_dir=outputs/train/panthera_diffusion \
  --wandb.enable=true \
  --wandb.project="lerobot panthera-HT" \
  --batch_size=4 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=100
```

#### 3. VQBeT (Vector Quantized Behavior Transformer)

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_gaoqing_test5 \
  --policy.type=vqbet \
  --policy.n_obs_steps=1 \
  --policy.chunk_size=100 \
  --output_dir=outputs/train/panthera_vqbet \
  --wandb.enable=true \
  --wandb.project="lerobot panthera-HT" \
  --batch_size=4 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=100
```

#### 4. TDMPC (Temporal Difference Model Predictive Control)

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_gaoqing_test5 \
  --policy.type=tdmpc \
  --output_dir=outputs/train/panthera_tdmpc \
  --wandb.enable=true \
  --wandb.project="lerobot panthera-HT" \
  --batch_size=4 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=100
```

**训练参数说明：**
- `--policy.type`: 训练算法类型
- `--batch_size`: 批次大小
- `--steps`: 训练步数
- `--save_freq`: 模型保存频率
- `--log_freq`: 日志记录频率
- `--wandb.enable`: 是否启用 Weights & Biases 日志

### 模型评估

使用训练好的模型进行评估：

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --policy.path=outputs/train/panthera_gaoqing_test/checkpoints/001000/pretrained_model \
  --dataset.repo_id=local/eval_gaoqing_test \
  --dataset.single_task="gaoqing_test" \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=5 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

**参数说明：**
- `--policy.path`: 训练好的模型路径
- 其他参数与数据采集类似

### 数据可视化

可视化已采集的数据集：

```bash
python lerobot_dataset_viz.py \
  --repo-id local/panthera_gaoqing_test \
  --episode-index 1 \
  --batch-size 8 \
  --output-dir ./viz_output
```

**参数说明：**
- `--repo-id`: 数据集 ID
- `--episode-index`: 要可视化的回合索引
- `--batch-size`: 批次大小
- `--output-dir`: 输出目录

---

## 支持的训练算法

LeRobot 支持以下 10 种训练算法：

| 算法 | 类型 | 主要超参数 | 适用场景 |
|------|------|-----------|---------|
| **act** | Action Chunking with Transformers | `n_obs_steps`, `n_action_steps`, `chunk_size` | 通用，适合长序列动作 |
| **diffusion** | Diffusion Policy | `n_obs_steps`, `horizon`, `n_action_steps` | 适合复杂动作分布 |
| **tdmpc** | Temporal Difference MPC | 主要使用默认参数 | 模型预测控制 |
| **vqbet** | Vector Quantized Behavior Transformer | `n_obs_steps`, `chunk_size` | 离散化动作空间 |
| **sac** | Soft Actor-Critic | 强化学习参数 | 在线强化学习 |
| **pi0** | PI0 Policy | 视觉语言模型参数 | 视觉语言任务 |
| **pi05** | PI0.5 Policy | 视觉语言模型参数 | 视觉语言任务 |
| **smolvla** | SmolVLA | 视觉语言模型参数 | 轻量级视觉语言 |
| **groot** | Groot Policy | 多模态参数 | 多模态任务 |
| **xvla** | XVLA Policy | 视觉语言模型参数 | 跨模态任务 |

### 算法切换方法

只需修改 `--policy.type` 参数即可切换算法。例如：

```bash
# 切换到 Diffusion Policy
python lerobot_train.py --policy.type=diffusion ...

# 切换到 VQBeT
python lerobot_train.py --policy.type=vqbet ...
```

### 查看算法特定参数

```bash
# 查看某个算法的所有可用参数
python lerobot_train.py --policy.type=diffusion --help
```

### 算法配置文件位置

每个算法的详细配置文件位于：
- `lerobot/src/lerobot/policies/act/configuration_act.py`
- `lerobot/src/lerobot/policies/diffusion/configuration_diffusion.py`
- `lerobot/src/lerobot/policies/vqbet/configuration_vqbet.py`
- 等等...

### 算法选择建议

1. **首次尝试新算法**：使用默认参数
2. **根据训练效果**：再调整超参数
3. **注意**：不同算法可能需要不同的 `batch_size` 和学习率

---

## 数据集结构说明

LeRobot 数据集采用标准化的目录结构：

```
local/eval_gaoqing_test5/
├── meta/                          # 元数据目录
│   ├── info.json                  # 数据集配置信息
│   ├── stats.json                 # 统计信息
│   ├── tasks.parquet              # 任务信息
│   └── episodes/                  # 记录片段的元数据
├── data/                          # 实际数据
│   └── *.parquet                  # Parquet 格式数据文件
└── videos/                        # 视频数据
    ├── observation.images.top/    # 顶部摄像头视频 (480x640, 30fps)
    └── observation.images.wrist/  # 手腕摄像头视频 (480x640, 30fps)
```

### 文件说明

- **info.json**: 包含机器人类型、帧率、特征定义等配置信息
- **stats.json**: 数据集的统计信息（均值、方差等）
- **tasks.parquet**: 任务描述和标签
- **data/*.parquet**: 存储机器人状态、动作等数值数据
- **videos/**: 存储观测视频数据

---

## 常见问题

### 1. 数据集路径配置问题

**问题描述：**
训练或可视化时出现路径错误，例如：
- `FileNotFoundError: .../local/panthera_gaoqing_test/local/panthera_gaoqing_test/meta/info.json`（路径重复）
- `FileNotFoundError: .../Data Collection Dataset/meta/info.json`（缺少 repo_id）

**原因：**
LeRobot 的路径处理逻辑需要正确配置。系统使用以下逻辑：
1. `default.py` 中的 `__post_init__` 方法会自动构建完整路径（包含 repo_id）
2. `LeRobotDataset` 和 `LeRobotDatasetMetadata` 直接使用提供的 root 路径
3. 如果路径处理不一致，会导致路径重复或缺失

**已修复的文件：**
- `lerobot/src/lerobot/configs/default.py:40-53` - 自动追加 repo_id 到 root 路径
- `lerobot/src/lerobot/datasets/lerobot_dataset.py:683` - 不额外追加 repo_id
- `lerobot/src/lerobot/scripts/lerobot_dataset_viz.py:270-272` - 在调用前追加 repo_id

**验证修复：**
```bash
# 训练命令应该正常工作
python lerobot_train.py --dataset.repo_id=local/panthera_gaoqing_test ...

# 可视化命令应该正常工作
python lerobot_dataset_viz.py --repo-id local/panthera_gaoqing_test ...
```

**数据集目录结构：**
```
Panthera-HT_lerobot_dataset/
└── Data Collection Dataset/
    └── local/
        └── panthera_gaoqing_test/    ← 完整的数据集路径
            ├── meta/
            │   ├── info.json
            │   ├── stats.json
            │   └── tasks.parquet
            ├── data/
            └── videos/
```

### 2. 为什么会运行错误路径的代码？

**问题描述：**
运行 `python lerobot_record.py --teleop.type=panthera_leader` 时，Python 加载了错误路径的代码。

**原因：**
Python 按照 `sys.path` 的顺序搜索模块：
1. 首先搜索已安装的包（通过 `pip install -e` 安装的）
2. 然后搜索环境变量 `PYTHONPATH` 中的路径
3. 最后搜索当前目录

如果之前在其他目录（如 `/home/sunteng/Panthera-HT_lerobot/`）执行过 `pip install -e .`，Python 会优先加载那个路径的代码。

**解决方案 1：重新安装当前路径的 lerobot（推荐）**

```bash
cd /home/sunteng/桌面/Panthera-HT_LeRobot/Panthera-HT_lerobot/lerobot
pip uninstall lerobot -y
pip install -e .
```

这会更新 `.pth` 文件，指向当前路径。

**解决方案 2：直接修改 .pth 文件**

找到并编辑文件：
```
/home/sunteng/anaconda3/envs/lerobot/lib/python3.10/site-packages/__editable__.lerobot-0.4.3.pth
```

将路径修改为当前工作目录。

### 3. 不同算法的超参数差异

不同算法有不同的关键超参数：

| 算法 | 关键超参数 |
|------|-----------|
| ACT | `n_obs_steps`, `n_action_steps`, `chunk_size` |
| Diffusion | `n_obs_steps`, `horizon`, `n_action_steps` |
| VQBeT | `n_obs_steps`, `chunk_size` |
| TDMPC | 主要使用默认参数 |

**建议：**
- 首次尝试新算法时，先使用默认参数
- 根据训练效果再调整超参数
- 不同算法可能需要不同的 `batch_size` 和学习率

### 3. 如何查看所有可用参数？

```bash
python lerobot_train.py --policy.type=<算法名> --help
```

例如：
```bash
python lerobot_train.py --policy.type=diffusion --help
```

---

## 相机配置

### 相机序列号说明

每个 RealSense 相机都有唯一的出厂序列号（如 `352122273105`），类似于设备的"身份证号"。

### 检测相机序列号

LeRobot 提供了专门的工具来查找相机序列号：

```bash
# 方法1：只查找 RealSense 相机
lerobot-find-cameras realsense

# 方法2：查找所有相机（包括 OpenCV 和 RealSense）
lerobot-find-cameras

# 方法3：查找并保存测试图片
lerobot-find-cameras realsense --output-dir ./test_images --record-time-s 2
```

**输出示例：**
```
--- Detected Cameras ---
Camera #0:
  Type: RealSense
  Id: 352122273105          ← 这就是序列号
  Name: Intel RealSense D405
  Default stream profile:
    Width: 640
    Height: 480
    Fps: 30
--------------------
Camera #1:
  Type: RealSense
  Id: 948122071707          ← 这就是序列号
  Name: Intel RealSense D435
  ...
```

### 更换相机后的配置步骤

如果更换了新相机，需要：

1. **运行检测命令：**
   ```bash
   lerobot-find-cameras realsense
   ```

2. **记录新的序列号**（例如：`123456789012`）

3. **修改配置文件：**

   编辑 `lerobot_robot_panthera/src/lerobot_robot_panthera/panthera_follower.py:40-43`：

   ```python
   camera_map = {
       "123456789012": "wrist",  # 新的 D405 序列号
       "948122071707": "top"     # 保持不变
   }
   ```

4. **更新 fallback 配置（第 77-94 行）：**

   ```python
   "wrist": RealSenseCameraConfig(
       serial_number_or_name="123456789012",  # 更新这里
       ...
   )
   ```

### 相机型号选择

- **D405**：短距离相机（工作距离 7-50cm），适合 **wrist**（手腕）位置
- **D435**：标准距离相机（工作距离 30-300cm），适合 **top**（顶部俯视）位置

---

## 其他资源

- **配置文件位置**: `lerobot/src/lerobot/policies/*/configuration_*.py`
- **机器人配置**: `lerobot_robot_panthera/src/lerobot_robot_panthera/`
- **训练日志**: 使用 Weights & Biases (wandb) 查看训练过程

---

## 许可证

请参考项目根目录的 LICENSE 文件。

---

## 贡献

欢迎提交 Issue 和 Pull Request！
