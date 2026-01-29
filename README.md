# Panthera-HT LeRobot 项目

本项目包含 Panthera-HT 机器人与 LeRobot 框架的集成开发环境。

## 目录结构

本目录包含以下三个主要文件夹：

### 📁 lerobot
LeRobot 框架的核心代码库，提供机器人学习和控制的基础功能。这是一个完整的 LeRobot 框架实现，包含：
- 机器人控制算法
- 数据采集工具
- 模型训练框架
- 示例代码和文档

### 📁 Panthera-HT_robot_lerobot
Panthera-HT 机器人与 LeRobot 框架的集成包，实现了 Panthera-HT 机器人在 LeRobot 框架下的接口适配。

```
Panthera-HT_robot_lerobot/
├── docs/                           # 文档目录
│   ├── README.md                  # 项目说明文档
│   └── Panthera_LeRobot_Integration_Guide.md  # 集成指南
├── src/                            # 源代码
│   └── lerobot_robot_panthera/    # Panthera 机器人接口实现
└── pyproject.toml                  # Python 项目配置
```

**📖 重要文档：**
- [集成指南](Panthera-HT_robot_lerobot/docs/Panthera_LeRobot_Integration_Guide.md) - 详细的集成步骤和使用说明
- [项目说明](Panthera-HT_robot_lerobot/docs/README.md) - 项目概述和快速开始

### 📁 Panthera-HT_lerobot_dataset
数据集目录，包含用于训练和评估的机器人数据。

```
Panthera-HT_lerobot_dataset/
├── Data Collection Dataset/        # 数据采集数据集
└── Model evaluation dataset/       # 模型评估数据集
```

## 快速开始

### 环境准备

1. 确保已安装 Python 3.9 或更高版本
2. 安装 LeRobot 框架依赖（参考 `lerobot` 目录中的文档）
3. 安装 Panthera-HT 机器人集成包

### 获取 Panthera-HT_SDK

本项目需要配合 Panthera-HT_SDK 使用。SDK 应放置在与本目录（Panthera-HT_lerobot）并行的位置。

如果您的环境中还没有 `Panthera-HT_SDK` 文件夹，请从 Git 仓库克隆：

```bash
# 进入上级目录（Panthera-HT_LeRobot）
cd ..

# 克隆 SDK（与 Panthera-HT_lerobot 并行）
git clone <Panthera-HT_SDK仓库地址> Panthera-HT_SDK
```

预期的目录结构：
```
Panthera-HT_LeRobot/
├── Panthera-HT_lerobot/    # 本目录
└── Panthera-HT_SDK/         # SDK 目录（需要克隆）
```

### 使用指南

1. 首先确保已安装 `Panthera-HT_SDK`
2. 查看 SDK 示例脚本了解基本的机器人控制方法
3. 阅读 LeRobot 框架文档，了解基本概念和使用方法
4. 查看 Panthera-HT 集成文档：
   - [Panthera_LeRobot_Integration_Guide.md](Panthera-HT_robot_lerobot/docs/Panthera_LeRobot_Integration_Guide.md) - 详细的集成指南
   - [README.md](Panthera-HT_robot_lerobot/docs/README.md) - 项目说明文档
5. 使用数据集进行模型训练和评估
6. 开始使用 LeRobot 框架进行机器人学习和控制

## 联系与支持

如有问题，请参考各子目录中的文档或联系项目维护者。
