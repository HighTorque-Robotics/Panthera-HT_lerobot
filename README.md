# Panthera-HT LeRobot Integration

[![中文](https://img.shields.io/badge/lang-简体中文-red.svg)](#中文)[![en](https://img.shields.io/badge/lang-English-blue.svg)](#english)

<a id="中文"></a>

Panthera-HT LeRobot Integration 是 Panthera-HT 机械臂接入 LeRobot 框架的示例工程，用于数据采集、模仿学习训练和策略评估。项目保留原生 `lerobot/` 框架源码，并在 `Panthera-HT_robot_lerobot/` 中提供 Panthera-HT 的机器人适配层。

### 项目结构

```text
Panthera-HT_lerobot/
├── lerobot/                         # LeRobot upstream framework source
├── Panthera-HT_robot_lerobot/       # Panthera-HT adapter package
│   ├── docs/                        # Usage documents
│   ├── src/lerobot_robot_panthera/  # Robot, teleop, and motor bus adapters
│   └── pyproject.toml               # Python package metadata
├── Panthera-HT_lerobot_dataset/     # Local datasets, not recommended for release
└── README.md
```

### Panthera-HT 适配内容

- `panthera_follower`: Panthera-HT 从臂机器人接口，用于采集、回放和策略评估。
- `panthera_leader`: Panthera-HT 主臂遥操作接口，用于主从示教采集。
- `panthera_dual_follower`: 双从臂控制实现，适合协同任务扩展。
- `panthera_bus`: 基于 Panthera-HT SDK 的电机总线封装。

### 环境要求

- Python 3.10 推荐。
- Panthera-HT SDK 需要与本项目并行放置。
- RealSense 相机驱动需要正常安装。
- 机器人参数文件默认从 `../Panthera-HT_SDK/panthera_python/robot_param/` 查找。

期望目录结构：

```text
Panthera-HT/
├── Panthera-HT_lerobot/
└── Panthera-HT_SDK/
```

### 安装

```bash
conda create -n lerobot python=3.10
conda activate lerobot

cd Panthera-HT_lerobot
pip install -e lerobot
pip install -e Panthera-HT_robot_lerobot
pip install pynput pin pyrealsense2
```

验证相机：

```bash
lerobot-find-cameras realsense
```

### 快速使用

进入 LeRobot 脚本目录：

```bash
cd lerobot/src/lerobot/scripts
```

主从模式数据采集：

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --teleop.type=panthera_leader \
  --dataset.repo_id=local/panthera_demo \
  --dataset.single_task="panthera_demo_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

单臂手动示教采集：

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --dataset.repo_id=local/panthera_teaching_demo \
  --dataset.single_task="manual_teaching_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=20 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

训练 ACT 策略：

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=act \
  --output_dir=outputs/train/panthera_act \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=50000 \
  --save_freq=10000 \
  --log_freq=100
```

策略评估：

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --policy.path=outputs/train/panthera_act/checkpoints/050000/pretrained_model \
  --dataset.repo_id=local/eval_panthera_act \
  --dataset.single_task="eval_task" \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=30 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

更多命令见 [Panthera-HT_robot_lerobot/docs/README.md](Panthera-HT_robot_lerobot/docs/README.md)。

### 开源前建议

- 不要提交本地数据集、训练输出和模型权重。
- 建议在 `.gitignore` 中排除 `outputs/`、`Panthera-HT_lerobot_dataset/` 和模型权重文件。
- 根目录当前没有独立 `LICENSE` 文件，正式开源前请补充许可证文件。
- `lerobot/` 是上游框架源码，非必要不建议大改，方便后续同步上游。


---

# Panthera-HT LeRobot Integration

[![en](https://img.shields.io/badge/lang-English-blue.svg)](#english)[![中文](https://img.shields.io/badge/lang-简体中文-red.svg)](#中文)

<a id="english"></a>

Panthera-HT LeRobot Integration connects the Panthera-HT robotic arm to the LeRobot framework for data collection, imitation learning, and policy evaluation. The repository keeps the upstream `lerobot/` source tree and adds the Panthera-HT adapter package under `Panthera-HT_robot_lerobot/`.

### Project Layout

```text
Panthera-HT_lerobot/
├── lerobot/                         # LeRobot upstream framework source
├── Panthera-HT_robot_lerobot/       # Panthera-HT adapter package
│   ├── docs/                        # Usage documents
│   ├── src/lerobot_robot_panthera/  # Robot, teleop, and motor bus adapters
│   └── pyproject.toml               # Python package metadata
├── Panthera-HT_lerobot_dataset/     # Local datasets, not recommended for release
└── README.md
```

### Panthera-HT Adapters

- `panthera_follower`: follower robot interface for data collection, replay, and policy evaluation.
- `panthera_leader`: leader teleoperation interface for leader-follower demonstrations.
- `panthera_dual_follower`: dual follower implementation for coordinated tasks.
- `panthera_bus`: motor bus wrapper based on the Panthera-HT SDK.

### Requirements

- Python 3.10 is recommended.
- The Panthera-HT SDK should be placed next to this repository.
- RealSense camera drivers must be installed and working.
- Robot parameter files are searched under `../Panthera-HT_SDK/panthera_python/robot_param/` by default.

Expected layout:

```text
Panthera-HT/
├── Panthera-HT_lerobot/
└── Panthera-HT_SDK/
```

### Installation

```bash
conda create -n lerobot python=3.10
conda activate lerobot

cd Panthera-HT_lerobot
pip install -e lerobot
pip install -e Panthera-HT_robot_lerobot
pip install pynput pin pyrealsense2
```

Check RealSense cameras:

```bash
lerobot-find-cameras realsense
```

### Quick Start

Go to the LeRobot scripts directory:

```bash
cd lerobot/src/lerobot/scripts
```

Leader-follower data collection:

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --teleop.type=panthera_leader \
  --dataset.repo_id=local/panthera_demo \
  --dataset.single_task="panthera_demo_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

Single-arm manual teaching:

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --dataset.repo_id=local/panthera_teaching_demo \
  --dataset.single_task="manual_teaching_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=20 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

Train an ACT policy:

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=act \
  --output_dir=outputs/train/panthera_act \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=50000 \
  --save_freq=10000 \
  --log_freq=100
```

Evaluate a policy:

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --policy.path=outputs/train/panthera_act/checkpoints/050000/pretrained_model \
  --dataset.repo_id=local/eval_panthera_act \
  --dataset.single_task="eval_task" \
  --dataset.num_episodes=5 \
  --dataset.episode_time_s=30 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

See [Panthera-HT_robot_lerobot/docs/README.md](Panthera-HT_robot_lerobot/docs/README.md) for detailed commands.

### Before Open Sourcing

- Do not commit local datasets, training outputs, or model weights.
- Add `outputs/`, `Panthera-HT_lerobot_dataset/`, and model weight files to `.gitignore` before publishing.
- The repository root currently has no standalone `LICENSE`; add one before release.
- Keep upstream `lerobot/` changes minimal so future upstream syncs remain manageable.
# Panthera_lerobot_HT
