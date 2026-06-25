# Panthera-HT LeRobot Usage Guide

[中文](#中文) | [English](#english)

<a id="中文"></a>
## 中文

本文档说明 Panthera-HT 适配 LeRobot 后的常用操作，包括环境安装、数据采集、训练、评估、数据可视化和相机配置。

### 1. 环境安装

创建环境：

```bash
conda create -n lerobot python=3.10
conda activate lerobot
```

安装 LeRobot 和 Panthera-HT 适配包：

```bash
cd ~/Panthera-HT/Panthera-HT_lerobot
pip install -e lerobot
pip install -e Panthera-HT_robot_lerobot
```

安装常用依赖：

```bash
pip install pynput pin pyrealsense2
```

验证安装：

```bash
lerobot-find-cameras realsense
cd lerobot/src/lerobot/scripts
python lerobot_record.py --help
```

### 2. 运行目录

下面的采集、训练、评估命令默认在 LeRobot 脚本目录运行：

```bash
cd ~/Panthera-HT/Panthera-HT_lerobot/lerobot/src/lerobot/scripts
```

### 3. 数据采集

#### 3.1 主从模式

适用于一条 Leader 臂和一条 Follower 臂。Leader 由人工拖动，Follower 跟随动作并记录机器人状态和相机图像。

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --teleop.type=panthera_leader \
  --dataset.repo_id=local/panthera_demo \
  --dataset.single_task="panthera_demo_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=60 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

关键参数：

- `--robot.type=panthera_follower`: 使用 Panthera-HT 从臂作为执行机器人。
- `--teleop.type=panthera_leader`: 启用 Panthera-HT 主臂遥操作。
- `--dataset.repo_id`: 本地数据集 ID，例如 `local/panthera_demo`。
- `--dataset.num_episodes`: 采集回合数。
- `--dataset.episode_time_s`: 每个回合录制时长。
- `--dataset.reset_time_s`: 两个回合之间的复位时间。

录制结束后，Follower 会进入位置保持模式，方便人工复位环境并开始下一回合。

#### 3.2 单臂手动示教模式

适用于只有一条 Panthera-HT 机械臂的场景。不要传 `--teleop.type`，机器人会进入示教模式，只做重力补偿，用户可以直接拖动机械臂完成示教。

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --dataset.repo_id=local/panthera_teaching_demo \
  --dataset.single_task="manual_teaching_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=60 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

切换规则很简单：

- 带 `--teleop.type=panthera_leader` 是主从模式。
- 不带 `--teleop.type` 是单臂手动示教模式。

### 4. 模型训练

ACT 示例：

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=act \
  --policy.n_obs_steps=1 \
  --policy.n_action_steps=50 \
  --policy.chunk_size=50 \
  --policy.use_vae=true \
  --policy.kl_weight=10.0 \
  --policy.optimizer_lr=1e-4 \
  --dataset.image_transforms.enable=true \
  --dataset.image_transforms.max_num_transforms=3 \
  --dataset.image_transforms.random_order=false \
  --output_dir=outputs/train/panthera_act \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=50000 \
  --save_freq=10000 \
  --log_freq=100
```

Diffusion Policy 示例：

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=diffusion \
  --policy.n_obs_steps=2 \
  --policy.horizon=16 \
  --policy.n_action_steps=8 \
  --output_dir=outputs/train/panthera_diffusion \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=100
```

常用训练参数：

- `--policy.type`: 策略类型，例如 `act`、`diffusion`、`vqbet`。
- `--batch_size`: 训练批次大小。
- `--steps`: 训练步数。
- `--save_freq`: 保存 checkpoint 的间隔。
- `--log_freq`: 打印日志的间隔。
- `--wandb.enable`: 是否启用 Weights & Biases。

### 5. 策略评估

使用训练好的模型控制机器人，并记录评估数据：

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

### 6. 数据可视化

```bash
python lerobot_dataset_viz.py \
  --repo-id local/panthera_demo \
  --episode-index 0 \
  --batch-size 8 \
  --output-dir ./viz_output
```

### 7. 相机配置

适配代码会自动检测 RealSense 相机序列号，并按以下默认规则命名：

| Serial | Camera name | Purpose |
| --- | --- | --- |
| `352122273105` | `right_wrist` or `wrist` | 右腕相机 |
| `352122272797` | `left_wrist` | 左腕相机 |
| `408322072614` | `top` | 顶视相机 |
| `335222076820` | `side` | 侧视相机 |

如果你的相机序列号不同，请修改 `Panthera-HT_robot_lerobot/src/lerobot_robot_panthera/panthera_follower.py` 和 `panthera_dual_follower.py` 中的默认序列号。

### 8. 输出和数据集

- 训练输出通常写入 `lerobot/src/lerobot/scripts/outputs/`。
- 本地数据集通常写入 `Panthera-HT_lerobot_dataset/` 或 LeRobot 默认数据目录。
- 开源发布时不要提交数据集、checkpoint、训练日志和大文件权重。

<a id="english"></a>
## English

This document explains common Panthera-HT workflows with LeRobot: environment setup, data collection, training, evaluation, dataset visualization, and camera configuration.

### 1. Environment Setup

Create the environment:

```bash
conda create -n lerobot python=3.10
conda activate lerobot
```

Install LeRobot and the Panthera-HT adapter package:

```bash
cd ~/Panthera-HT/Panthera-HT_lerobot
pip install -e lerobot
pip install -e Panthera-HT_robot_lerobot
```

Install common dependencies:

```bash
pip install pynput pin pyrealsense2
```

Verify the installation:

```bash
lerobot-find-cameras realsense
cd lerobot/src/lerobot/scripts
python lerobot_record.py --help
```

### 2. Working Directory

The following data collection, training, and evaluation commands assume this directory:

```bash
cd ~/Panthera-HT/Panthera-HT_lerobot/lerobot/src/lerobot/scripts
```

### 3. Data Collection

#### 3.1 Leader-Follower Mode

Use this mode with one leader arm and one follower arm. The human moves the leader arm, while the follower arm tracks the motion and records robot states and camera images.

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --teleop.type=panthera_leader \
  --dataset.repo_id=local/panthera_demo \
  --dataset.single_task="panthera_demo_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=30 \
  --dataset.reset_time_s=60 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

Key parameters:

- `--robot.type=panthera_follower`: uses the Panthera-HT follower arm as the robot.
- `--teleop.type=panthera_leader`: enables the Panthera-HT leader teleoperator.
- `--dataset.repo_id`: local dataset ID, for example `local/panthera_demo`.
- `--dataset.num_episodes`: number of episodes to record.
- `--dataset.episode_time_s`: recording duration per episode.
- `--dataset.reset_time_s`: reset time between episodes.

After each recording episode, the follower enters position hold mode so the operator can reset the environment before the next episode.

#### 3.2 Single-Arm Manual Teaching Mode

Use this mode when only one Panthera-HT arm is available. Do not pass `--teleop.type`; the robot enters teaching mode with gravity compensation only, so the user can manually move the arm for demonstrations.

```bash
python lerobot_record.py \
  --robot.type=panthera_follower \
  --dataset.repo_id=local/panthera_teaching_demo \
  --dataset.single_task="manual_teaching_task" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=20 \
  --dataset.reset_time_s=60 \
  --dataset.push_to_hub=false \
  --display_data=false \
  --play_sounds=false
```

Mode selection:

- With `--teleop.type=panthera_leader`: leader-follower mode.
- Without `--teleop.type`: single-arm manual teaching mode.

### 4. Model Training

ACT example:

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=act \
  --policy.n_obs_steps=1 \
  --policy.n_action_steps=50 \
  --policy.chunk_size=50 \
  --policy.use_vae=true \
  --policy.kl_weight=10.0 \
  --policy.optimizer_lr=1e-4 \
  --dataset.image_transforms.enable=true \
  --dataset.image_transforms.max_num_transforms=3 \
  --dataset.image_transforms.random_order=false \
  --output_dir=outputs/train/panthera_act \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=50000 \
  --save_freq=10000 \
  --log_freq=100
```

Diffusion Policy example:

```bash
python lerobot_train.py \
  --dataset.repo_id=local/panthera_demo \
  --policy.type=diffusion \
  --policy.n_obs_steps=2 \
  --policy.horizon=16 \
  --policy.n_action_steps=8 \
  --output_dir=outputs/train/panthera_diffusion \
  --wandb.enable=false \
  --batch_size=4 \
  --steps=100000 \
  --save_freq=10000 \
  --log_freq=100
```

Common training parameters:

- `--policy.type`: policy type, such as `act`, `diffusion`, or `vqbet`.
- `--batch_size`: training batch size.
- `--steps`: number of training steps.
- `--save_freq`: checkpoint save interval.
- `--log_freq`: log print interval.
- `--wandb.enable`: whether to enable Weights & Biases.

### 5. Policy Evaluation

Run a trained policy on the robot and record evaluation data:

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

### 6. Dataset Visualization

```bash
python lerobot_dataset_viz.py \
  --repo-id local/panthera_demo \
  --episode-index 0 \
  --batch-size 8 \
  --output-dir ./viz_output
```

### 7. Camera Configuration

The adapter auto-detects RealSense serial numbers and assigns camera names with these defaults:

| Serial | Camera name | Purpose |
| --- | --- | --- |
| `352122273105` | `right_wrist` or `wrist` | right wrist camera |
| `352122272797` | `left_wrist` | left wrist camera |
| `408322072614` | `top` | top-view camera |
| `335222076820` | `side` | side-view camera |

If your serial numbers differ, update the defaults in `Panthera-HT_robot_lerobot/src/lerobot_robot_panthera/panthera_follower.py` and `panthera_dual_follower.py`.

### 8. Outputs and Datasets

- Training outputs are usually written to `lerobot/src/lerobot/scripts/outputs/`.
- Local datasets are usually written to `Panthera-HT_lerobot_dataset/` or the default LeRobot dataset path.
- Do not publish datasets, checkpoints, training logs, or large weight files in the open-source repository.
