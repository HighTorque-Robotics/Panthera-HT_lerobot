from dataclasses import dataclass
import os
import yaml
import logging
import time
import threading
import numpy as np
import pinocchio as pin  # 机器人动力学库

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .motors.panthera_motors_bus import PantheraMotorsBus

# ============================== 全局配置 ==============================
# 创建日志器，用于记录Panthera Leader遥操作器的运行日志
logger = logging.getLogger(__name__)

# ============================== 配置类 ==============================
@TeleoperatorConfig.register_subclass("panthera_leader")  # 注册为新名字的遥操作器
@dataclass
class PantheraLeaderConfig(TeleoperatorConfig):
    """
    Panthera Leader遥操作器的配置类
    
    该类继承自TeleoperatorConfig，用于存储Panthera机械臂Leader端的所有配置参数
    主要包括配置文件路径和摩擦补偿相关参数
    """
    # Panthera特定的配置文件路径（例如Leader.yaml），包含机器人运动学参数
    param_path: str = "robot_param/Leader.yaml"
    
    # 摩擦补偿参数（SDK示例的默认值），按关节顺序排列
    # Fc: 库仑摩擦系数，Fv: 粘性摩擦系数
    fc: tuple = (0.20, 0.15, 0.15, 0.15, 0.04, 0.04)  # 库仑摩擦系数
    fv: tuple = (0.06, 0.06, 0.06, 0.03, 0.02, 0.02)  # 粘性摩擦系数
    vel_threshold: float = 0.02  # 速度阈值，用于判断是否应用库仑摩擦补偿

# ============================== 核心类 ==============================
class PantheraLeader(Teleoperator):
    """
    Panthera Leader遥操作器实现类

    该类实现了与Panthera机械臂Leader端的通信、状态获取和重力/摩擦补偿控制
    主要功能包括：
    1. 连接/断开Panthera电机总线
    2. 实时获取关节位置信息
    3. 后台线程进行重力和摩擦补偿
    4. 夹爪状态管理（如果有）
    """
    # 指定配置类和遥操作器名称
    config_class = PantheraLeaderConfig
    name = "panthera_leader"

    def __init__(self, config: PantheraLeaderConfig):
        """
        初始化Panthera Leader遥操作器
        
        Args:
            config: PantheraLeaderConfig配置实例，包含参数文件路径和补偿参数
        """
        super().__init__(config)  # 调用父类的初始化方法
        self.config = config

        # ============================== 配置文件路径处理 ==============================
        # 解析配置文件路径，支持相对路径和绝对路径
        param_path = config.param_path
        if not os.path.isabs(param_path):  # 如果不是绝对路径
            if not os.path.exists(param_path):  # 如果相对路径不存在
                # 尝试从Panthera SDK目录查找配置文件
                # __file__ is the current file path
                curr_file = os.path.abspath(__file__)
                # /home/sunteng/桌面/Panthera-HT_LeRobot/Panthera-HT_lerobot/Panthera-HT_robot_lerobot/src/lerobot_robot_panthera/panthera_leader.py
                # Go up 5 levels: panthera_leader.py -> lerobot_robot_panthera -> src -> Panthera-HT_robot_lerobot -> Panthera-HT_lerobot -> Panthera-HT_LeRobot
                panthera_ht_lerobot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_file)))))
                # Now go to Panthera-HT_SDK/panthera_python
                panthera_sdk_path = os.path.join(panthera_ht_lerobot, "Panthera-HT_SDK", "panthera_python", param_path)
                logger.info(f"查找配置文件: {panthera_sdk_path}")
                if os.path.exists(panthera_sdk_path):  # 如果路径存在，则使用它
                    param_path = panthera_sdk_path
                    logger.info(f"找到配置文件: {param_path}")
                else:
                    logger.warning(f"配置文件不存在: {panthera_sdk_path}")
        
        # ============================== 电机总线初始化 ==============================
        # 创建与Panthera电机的总线连接实例
        self.bus = PantheraMotorsBus(param_path)

        # ============================== 关节名称加载 ==============================
        # 从YAML配置文件加载关节名称列表
        self.joint_names = []
        try:
            with open(param_path, 'r', encoding='utf-8') as f:
                robot_yaml = yaml.safe_load(f)  # 解析YAML配置文件
                # 从kinematics节点获取joint_names，默认空列表
                self.joint_names = robot_yaml.get('kinematics', {}).get('joint_names', [])
        except Exception as e:
            logger.error(f"加载机器人配置失败: {e}")  # 记录配置加载错误
            
        if not self.joint_names:
            # 如果未找到关节名称，给出警告，后续会使用默认命名
            logger.warning("未在配置中找到关节名称，默认使用motor_0..N")

        # ============================== 预先添加夹爪 ==============================
        # 在 action_features 被调用之前就添加夹爪，确保数据集特征包含夹爪
        if len(self.joint_names) == 6:  # 如果只有 6 个关节，预先添加夹爪
            self.joint_names.append("gripper")
            logger.info("预先添加夹爪到 joint_names，确保 action_features 包含夹爪")

        # ============================== 映射关系初始化 ==============================
        self.motor_map_inv = {}  # 电机名称到关节名称的反向映射 {joint_name: motor_name}
        self.motor_map = {}      # 关节名称到电机名称的正向映射 {motor_name: joint_name}
        
        # ============================== 线程控制初始化 ==============================
        self._thread = None               # 重力补偿后台线程实例
        self._stop_event = threading.Event()  # 线程停止事件，用于优雅停止后台线程
        self._latest_action = {}          # 存储最新的关节位置动作数据
        self._action_lock = threading.Lock()  # 线程锁，保护_latest_action的读写
        
        # ============================== 补偿参数初始化 ==============================
        # 将配置中的补偿参数转换为numpy数组，方便后续计算
        self.fc = np.array(config.fc)             # 库仑摩擦系数数组
        self.fv = np.array(config.fv)             # 粘性摩擦系数数组
        self.vel_threshold = config.vel_threshold # 摩擦补偿速度阈值

    @property
    def action_features(self):
        """
        获取动作特征定义（属性方法）
        
        返回值定义了遥操作器可以输出的动作特征类型，这里主要是各关节的位置
        格式: {特征名: 数据类型}
        
        Returns:
            dict: 关节位置特征字典，例如 {"joint1.pos": float, "joint2.pos": float}
        """
        # 为每个关节创建pos特征，数据类型为float
        return {f"{name}.pos": float for name in self.joint_names}

    @property
    def feedback_features(self):
        """
        获取反馈特征定义（属性方法）
        
        返回值定义了遥操作器可以接收的反馈特征类型，当前版本未实现力反馈
        
        Returns:
            dict: 空字典，表示暂无反馈特征
        """
        # 目前没有力反馈的实现，返回空字典
        return {}

    @property
    def is_connected(self):
        """
        检查设备连接状态（属性方法）
        
        Returns:
            bool: True表示已连接，False表示未连接
        """
        # 代理调用总线的连接状态检查
        return self.bus.is_connected

    @property
    def is_calibrated(self):
        """
        检查设备校准状态（属性方法）
        
        Panthera机械臂默认已完成校准，无需用户手动校准
        
        Returns:
            bool: 始终返回True
        """
        # 默认认为机器人已经校准完成
        return True

    def connect(self, calibrate=True):
        """
        连接Panthera Leader设备，初始化电机映射，并启动重力补偿线程
        
        该方法是设备使用的入口，会完成以下操作：
        1. 检查连接状态，避免重复连接
        2. 建立电机总线连接
        3. 构建电机与关节的映射关系
        4. 检测并处理夹爪电机
        5. 启动重力补偿后台线程
        
        Args:
            calibrate: 是否在校准后连接（兼容父类接口，实际无作用）
        
        Raises:
            DeviceAlreadyConnectedError: 设备已连接时抛出该异常
        """
        # 检查是否已连接，避免重复连接
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} 已经连接")
            
        # ============================== 建立电机总线连接 ==============================
        self.bus.connect()
        
        # ============================== 构建电机-关节映射 ==============================
        # 重置映射字典
        self.motor_map_inv = {} 
        self.motor_map = {} 
        
        # 为每个关节建立与电机的映射关系
        for i, name in enumerate(self.joint_names):
            if i < len(self.bus.motor_names):  # 确保不越界
                m_name = self.bus.motor_names[i]  # 获取对应索引的电机名称
                self.motor_map_inv[name] = m_name  # 关节名->电机名
                self.motor_map[m_name] = name      # 电机名->关节名
        
        # ============================== 检测夹爪电机 ==============================
        # 检查是否有额外的电机（通常是夹爪）
        # 注意：夹爪已经在 __init__ 中预先添加到 joint_names 了，这里只需要建立映射
        if len(self.bus.htr_motors) > 6:  # 如果电机数量超过 6 个（有夹爪）
            gripper_name = "gripper"  # 夹爪默认名称
            gripper_idx = 6  # 夹爪电机在数组中的索引（第 7 个电机，索引为 6）

            if gripper_idx < len(self.bus.motor_names):
                m_name = self.bus.motor_names[gripper_idx]  # 获取夹爪电机名称
                logger.info(f"检测到夹爪电机 {m_name}，将其映射为'{gripper_name}'")
                # 建立夹爪的映射关系（不需要再 append，因为 __init__ 中已经添加了）
                self.motor_map_inv[gripper_name] = m_name
                self.motor_map[m_name] = gripper_name
                
        # ============================== 启动补偿线程 ==============================
        self._stop_event.clear()  # 清除停止事件（确保线程可以启动）
        # 创建后台线程，daemon=True表示主线程退出时自动终止
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()  # 启动线程
        
        logger.info(f"{self} 连接成功并启动了重力补偿线程。")

    def disconnect(self):
        """
        断开与Panthera Leader设备的连接，并停止重力补偿线程
        
        该方法会优雅地停止后台线程，然后断开电机总线连接，确保设备安全关闭
        """
        # 如果未连接，直接返回
        if not self.is_connected:
            return
            
        # ============================== 停止后台线程 ==============================
        if self._thread:
            self._stop_event.set()  # 设置停止事件，通知线程退出
            # 等待线程退出，超时2秒（避免线程卡死）
            self._thread.join(timeout=2.0)
            self._thread = None  # 清空线程引用
            
        # ============================== 断开总线连接 ==============================
        self.bus.disconnect()

    def calibrate(self):
        """
        校准设备（空实现）
        
        Panthera机械臂在出厂时已完成校准，用户无需手动校准
        该方法仅为兼容父类接口而存在
        """
        logger.info(f"{self} 不需要用户校准。")
        pass

    def configure(self):
        """
        配置设备（空实现）
        
        预留的配置方法，当前版本暂无需要配置的参数
        """
        pass

    def get_action(self):
        """
        获取机器人的最新关节位置动作信息
        
        该方法是遥操作器的核心接口，返回最新的关节位置数据
        优先返回后台线程更新的_latest_action，如果为空则同步读取
        
        Returns:
            dict: 关节位置字典，例如 {"joint1.pos": 0.1, "joint2.pos": 0.2}
        
        Raises:
            DeviceNotConnectedError: 设备未连接时抛出该异常
        """
        # 检查设备连接状态
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} 没有连接")
            
        # 使用线程锁保护共享数据的读取
        with self._action_lock:
            if not self._latest_action:
                # 如果线程还没有更新动作数据，尝试同步读取作为回退方案
                try:
                    # 同步读取所有电机的当前位置
                    motor_vals = self.bus.sync_read("Present_Position")
                    action_dict = {}
                    # 转换电机位置到关节位置
                    for m_name, val in motor_vals.items():
                        if m_name in self.motor_map:
                            j_name = self.motor_map[m_name]
                            action_dict[f"{j_name}.pos"] = val
                    return action_dict
                except Exception as e:
                    # 读取失败时返回默认值（所有关节位置为0）
                    logger.warning(f"同步读取电机位置失败，返回默认值: {e}")
                    return {f"{name}.pos": 0.0 for name in self.joint_names}
            
            # 返回最新动作数据的副本（避免外部修改影响内部数据）
            return self._latest_action.copy()

    def send_feedback(self, feedback):
        """
        发送反馈信号到设备（空实现）
        
        当前版本未实现力反馈功能，该方法仅为兼容父类接口
        
        Args:
            feedback: 反馈数据字典（未使用）
        """
        pass

    def _control_loop(self):
        """
        重力和摩擦补偿的后台控制循环（内部方法）

        该方法在独立线程中运行，主要功能：
        1. 以200Hz频率更新电机状态
        2. 计算重力补偿和摩擦补偿力矩
        3. 发送补偿力矩指令到电机，实现被动柔顺控制
        4. 更新最新关节位置数据供get_action使用
        5. 处理夹爪的补偿和控制
        """
        logger.info("启动重力补偿循环...")

        # 获取机器人实例（来自电机总线）
        robot = self.bus.robot

        # ============================== 预分配数组 ==============================
        # 预分配零值数组，避免循环内频繁创建对象，提升性能
        num_motors = robot.motor_count  # 获取电机总数
        zero_pos = [0.0] * num_motors   # 零位置数组
        zero_vel = [0.0] * num_motors   # 零速度数组
        zero_kp = [0.0] * num_motors    # 零位置增益数组
        zero_kd = [0.0] * num_motors    # 零速度增益数组

        # ============================== 力矩限制配置 ==============================
        # 各关节的力矩限制（根据Panthera机械臂特性设定）
        # 顺序：基座关节 -> 末端关节，夹爪单独处理
        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

        # ============================== 主循环 ==============================
        # 循环直到收到停止事件
        while not self._stop_event.is_set():
            try:
                # 1. 获取当前电机状态
                vel = robot.get_current_vel()  # 获取关节速度（rad/s）
                pos = robot.get_current_pos()  # 获取关节位置（rad）

                # 2. 处理夹爪位置（如果有）
                try:
                    gripper_pos = robot.get_current_pos_gripper()  # 获取夹爪位置
                    pos = np.append(pos, gripper_pos)  # 将夹爪位置添加到关节位置数组
                except Exception as e:
                    pass  # 无夹爪或获取失败时跳过

                # 3. 更新最新动作数据（供get_action使用）
                action = {}
                for i, name in enumerate(self.joint_names):
                    if i < len(pos):  # 确保不越界
                        action[f"{name}.pos"] = pos[i]  # 存储关节位置

                # 使用线程锁更新共享数据
                with self._action_lock:
                    self._latest_action = action

                # 4. 计算补偿力矩
                try:
                    # 4.1 计算重力补偿力矩（基于Pinocchio动力学计算）
                    tau_gravity = robot.get_Gravity()

                    # 4.2 计算摩擦补偿力矩
                    tau_friction = robot.get_friction_compensation(
                        vel=vel,                # 当前关节速度
                        Fc=self.fc,             # 库仑摩擦系数
                        Fv=self.fv,             # 粘性摩擦系数
                        vel_threshold=self.vel_threshold  # 速度阈值
                    )

                    # 4.3 总补偿力矩 = 重力补偿 + 摩擦补偿
                    tau_total = tau_gravity + tau_friction

                    # 4.4 应用力矩限制，防止过载
                    tau_limit = tau_limit_base
                    # 如果电机数量超过基础关节数（包含夹爪），扩展力矩限制数组
                    if len(tau_total) > len(tau_limit):
                        # 夹爪力矩限制设为5.0
                        tau_limit = np.append(tau_limit, [5.0] * (len(tau_total) - len(tau_limit)))
                    else:
                        # 截断到实际电机数量
                        tau_limit = tau_limit[:len(tau_total)]

                    # 将补偿力矩限制在安全范围内
                    tau_total = np.clip(tau_total, -tau_limit, tau_limit)

                except Exception as e:
                    # 补偿计算失败时，使用零力矩，避免设备异常
                    logger.warning(f"补偿力矩计算失败，使用零力矩: {e}")
                    tau_total = np.zeros(num_motors)

                # 5. 手动设置前6个关节的命令
                for i in range(6):
                    robot.Motors[i].pos_vel_tqe_kp_kd(zero_pos[i], zero_vel[i], tau_total[i], zero_kp[i], zero_kd[i])

                # 6. 手动设置夹爪命令 - 保持在1.6位置
                robot.Motors[6].pos_vel_tqe_kp_kd(1.6, 0.0, 0.0, 0.2, 0.02)

                # 7. 统一发送所有7个电机的命令
                robot.motor_send_cmd()

                # 8. 控制循环频率（200Hz = 5ms）
                time.sleep(0.005) 
                
            except Exception as e:
                # 捕获所有异常，确保后台线程不会崩溃
                logger.error(f"控制循环出错: {e}")
                # 出错时延长睡眠时间，减少错误日志刷屏
                time.sleep(0.1)
                
        # ============================== 清理操作 ==============================
        # 线程退出时，发送零力矩指令，确保设备安全
        try:
            # 重置所有电机力矩为零
            robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, [0.0]*num_motors, zero_kp, zero_kd)
            # 重置夹爪控制为零力矩
            robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning(f"清理操作失败: {e}")

        logger.info("重力补偿循环停止。")
