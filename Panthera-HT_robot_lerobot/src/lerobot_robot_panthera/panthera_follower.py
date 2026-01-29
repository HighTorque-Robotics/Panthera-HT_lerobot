import numpy as np
import yaml
import os
import sys
import logging
import time
import threading
from functools import cached_property
from dataclasses import dataclass, field

#  import lerobot

from lerobot.robots.robot import Robot
from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.cameras import CameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.robots.config import RobotConfig


from .motors.panthera_motors_bus import PantheraMotorsBus

logger = logging.getLogger(__name__)

def default_cameras_factory() -> dict[str, CameraConfig]:
    """
    Configures RealSense cameras based on specific serial numbers.
    - D405 (352122273105) -> images.wrist
    - D435 (948122071707) -> images.top
    """
    try:
        available_cameras = RealSenseCamera.find_cameras()
        cameras = {}
        
        # Target configuration map
        camera_map = {
            "352122273105": "wrist", # D405
            "948122071707": "top"    # D435
        }
        
        if available_cameras:
            found_serials = [c["id"] for c in available_cameras]
            logger.info(f"Found RealSense cameras: {found_serials}")
            
            for cam_info in available_cameras:
                serial = cam_info["id"]
                
                # Check if this serial is in our map
                if serial in camera_map:
                    key = camera_map[serial]
                    logger.info(f"Configuring camera {key} with serial {serial}")
                    
                    cameras[key] = RealSenseCameraConfig(
                        serial_number_or_name=serial,
                        fps=30,
                        width=640,
                        height=480,
                        color_mode=ColorMode.RGB,
                        use_depth=False,
                        rotation=Cv2Rotation.NO_ROTATION
                    )
                else:
                    logger.info(f"Camera {serial} not in map, skipping.")
            
            if cameras:
                return cameras

    except Exception as e:
        logger.warning(f"Failed to auto-detect RealSense cameras: {e}")

    # Fallback default if detection fails or no cameras found
    return {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="352122273105",
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION
        ),
        "top": RealSenseCameraConfig(
            serial_number_or_name="948122071707",
            fps=30,
            width=640,
            height=480,
            color_mode=ColorMode.RGB,
            use_depth=False,
            rotation=Cv2Rotation.NO_ROTATION
        )
    }

@RobotConfig.register_subclass("panthera_follower")
@dataclass
class PantheraFollowerConfig(RobotConfig):
    # Path to the Panthera specific config file (e.g. Follower.yaml)
    # Can be absolute or relative to project root
    param_path: str = "robot_param/Follower.yaml"
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras_factory)

class PantheraFollower(Robot):
    """
    Panthera Follower Robot implementation.
    """
    config_class = PantheraFollowerConfig
    name = "panthera_follower"

    def __init__(self, config):
        super().__init__(config)
        self.config = config

        # Resolve param path
        param_path = config.param_path
        if not os.path.isabs(param_path):
            if not os.path.exists(param_path):
                 # Try to find in Panthera SDK directory
                 # __file__ is the current file path
                 curr_file = os.path.abspath(__file__)
                 # /home/sunteng/桌面/lerobotPantheraHT/Panthera-HT_lerobot/Panthera-HT_robot_lerobot/src/lerobot_robot_panthera/panthera_follower.py
                 # Go up 5 levels: panthera_follower.py -> lerobot_robot_panthera -> src -> Panthera-HT_robot_lerobot -> Panthera-HT_lerobot -> Panthera-HT_LeRobot
                 panthera_ht_lerobot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_file)))))
                 # Now go to Panthera-HT_SDK/panthera_python
                 panthera_sdk_path = os.path.join(panthera_ht_lerobot, "Panthera-HT_SDK", "panthera_python", param_path)
                 logger.info(f"Looking for config at: {panthera_sdk_path}")
                 if os.path.exists(panthera_sdk_path):
                     param_path = panthera_sdk_path
                     logger.info(f"Found config at: {param_path}")
                 else:
                     logger.warning(f"Config not found at: {panthera_sdk_path}")
        
        self.bus = PantheraMotorsBus(param_path)
        
        # Load config to determine features (joint names)
        self.joint_names = []
        try:
            with open(param_path, 'r', encoding='utf-8') as f:
                robot_yaml = yaml.safe_load(f)
                self.joint_names = robot_yaml.get('kinematics', {}).get('joint_names', [])
        except Exception as e:
            logger.error(f"Failed to load robot config from {param_path}: {e}")
            
        if not self.joint_names:
             logger.warning("No joint names found in config, defaulting to motor_0..N if connected")
             pass

        # ============================== 预先添加夹爪 ==============================
        # 在 action_features 被调用之前就添加夹爪，确保数据集特征包含夹爪
        if len(self.joint_names) == 6:  # 如果只有 6 个关节，预先添加夹爪
            self.joint_names.append("gripper")
            logger.info("预先添加夹爪到 joint_names，确保 action_features 包含夹爪")

        self.cameras = make_cameras_from_configs(config.cameras)
        
        self.motor_map_inv = {} 
        self.motor_map = {} 
        
        # Initialize threading and state variables
        self._thread = None
        self._stop_event = threading.Event()
        self._obs_lock = threading.Lock()
        self._latest_motor_pos = None
        self._last_action_time = 0.0
        
        # Friction compensation parameters (defaults matching Leader)
        self.fc = getattr(config, "fc", (0.2, 0.15, 0.15, 0.15, 0.04, 0.04))
        self.fv = getattr(config, "fv", (0.06, 0.06, 0.06, 0.03, 0.02, 0.02))
        self.vel_threshold = getattr(config, "vel_threshold", 0.02)

    @property
    def _motors_ft(self):
        return {f"{name}.pos": float for name in self.joint_names}
        
    @property
    def _cameras_ft(self):
         features = {}
         for cam_key, cam_config in self.config.cameras.items():
             features[cam_key] = (cam_config.height, cam_config.width, 3)
             
             # Add depth feature if enabled (e.g. for RealSense)
             if getattr(cam_config, "use_depth", False):
                 features[f"{cam_key}_depth"] = (cam_config.height, cam_config.width,1)
                 
         return features

    @property
    def observation_features(self):
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self):
        return self._motors_ft
        
    @property
    def is_connected(self):
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
        
    def connect(self, calibrate=True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")
            
        self.bus.connect()
        
        # Verify motor count matches joint names
        if len(self.bus.htr_motors) < len(self.joint_names):
             logger.warning(f"Found {len(self.bus.htr_motors)} motors, but expected {len(self.joint_names)} joints.")
        
        # Map motor names (motor_0, ...) to joint names (joint1, ...)
        self.motor_map_inv = {}
        self.motor_map = {}

        # 建立电机与关节的映射关系（包括夹爪）
        for i, name in enumerate(self.joint_names):
            if i < len(self.bus.motor_names):
                m_name = self.bus.motor_names[i]
                self.motor_map_inv[name] = m_name
                self.motor_map[m_name] = name
                logger.info(f"映射关节 '{name}' 到电机 '{m_name}'")

        # 检查夹爪映射是否成功
        if "gripper" in self.joint_names:
            if "gripper" in self.motor_map_inv:
                logger.info(f"夹爪映射成功: gripper -> {self.motor_map_inv['gripper']}")
            else:
                logger.warning("夹爪在 joint_names 中，但未能映射到电机")

        # Connect cameras with error handling
        try:
            for cam in self.cameras.values():
                cam.connect()
        except Exception as e:
            # If camera connection fails, disconnect bus before raising
            logger.error(f"Failed to connect cameras: {e}")
            self.bus.disconnect()
            raise

        # Initialize target_positions to current state to avoid jumps
        try:
             current_pos = self.bus.robot.get_current_pos()
             if len(current_pos) == len(self.bus.target_positions):
                 self.bus.target_positions = np.array(current_pos)
        except Exception as e:
             logger.warning(f"Failed to initialize target positions: {e}")

        # Start gravity compensation thread
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        logger.info(f"{self} connected.")

    def disconnect(self):
        if not self.is_connected:
            return
            
        # Stop thread
        if self._thread:
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._thread = None
            
        self.bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self):
        # Panthera robots are assumed to be pre-calibrated or have absolute encoders
        return True

    def calibrate(self):
        # No-op for Panthera
        logger.info(f"{self} does not require user-side calibration.")
        pass

    def configure(self):
        # No-op for Panthera
        pass

    def get_observation(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
            
        motor_vals = self.bus.sync_read("Present_Position")
        
        obs_dict = {}
        for m_name, val in motor_vals.items():
            if m_name in self.motor_map:
                j_name = self.motor_map[m_name]
                obs_dict[f"{j_name}.pos"] = val
                
        for cam_key, cam in self.cameras.items():
            # Retry logic for async_read
            max_retries = 3
            frame = None
            for attempt in range(max_retries):
                try:
                    frame = cam.async_read()
                    break
                except TimeoutError:
                    if attempt == max_retries - 1:
                        logger.error(f"Timeout reading from {cam_key} after {max_retries} attempts.")
                        raise
                    # logger.warning(f"Timeout reading from {cam_key}, retrying ({attempt + 1}/{max_retries})...")
                    time.sleep(0.005) # Short wait before retry
                except Exception as e:
                    logger.error(f"Error reading from {cam_key}: {e}")
                    raise e
            
            obs_dict[cam_key] = frame
            
            # Check for depth if using RealSense
            if isinstance(cam, RealSenseCamera) and getattr(cam, "use_depth", False):
                try:
                    # RealSense depth read is synchronous
                    depth_map = cam.read_depth()
                    # Add channel dimension (H, W) -> (H, W, 1)
                    if depth_map.ndim == 2:
                        depth_map = np.expand_dims(depth_map, axis=-1)
                    obs_dict[f"{cam_key}_depth"] = depth_map
                except Exception as e:
                    logger.warning(f"Failed to read depth from {cam_key}: {e}")
            
        return obs_dict

    def send_action(self, action):
        self._last_action_time = time.time()

        # Prepare full motor command arrays
        robot = self.bus.robot
        all_motors = robot.Motors
        total_motors = len(all_motors)

        # Use existing target positions as baseline to avoid jumping unactuated motors
        # Ensure target_pos has enough elements for all motors
        if len(self.bus.target_positions) < total_motors:
             # Should not happen if bus initialized correctly, but safety first
             target_pos = list(self.bus.target_positions) + [0.0] * (total_motors - len(self.bus.target_positions))
        else:
             target_pos = list(self.bus.target_positions)

        target_vel = [0.0] * total_motors

        # Calculate Follower Gravity for Feedforward Torque
        try:
            tau_gravity = robot.get_Gravity() # Returns 6 values for arm

            # Torque limits
            tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
            # Handle dimension mismatch for arm
            if len(tau_gravity) > len(tau_limit):
                pad_width = len(tau_gravity) - len(tau_limit)
                tau_limit = np.append(tau_limit, [5.0] * pad_width)
            elif len(tau_gravity) < len(tau_limit):
                tau_limit = tau_limit[:len(tau_gravity)]

            tau_arm = np.clip(tau_gravity, -tau_limit, tau_limit)

            # Construct full torque array (Arm + Gripper 0.0)
            target_tqe = list(tau_arm)
            if len(target_tqe) < total_motors:
                target_tqe.extend([0.0] * (total_motors - len(target_tqe)))

        except Exception as e:
            logger.warning(f"Failed to get gravity for compensation: {e}")
            target_tqe = [0.0] * total_motors

        # Kp, Kd defaults (Arm + Gripper)
        # Added stiff gains for gripper at index 6
        kp_defaults = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0, 4.0]
        kd_defaults = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1, 0.4]

        kp = [0.0] * total_motors
        kd = [0.0] * total_motors

        for i in range(total_motors):
            if i < len(kp_defaults):
                kp[i] = kp_defaults[i]
                kd[i] = kd_defaults[i]
            else:
                # Fallback for extra motors
                kp[i] = 1.0
                kd[i] = 0.1

        # Map action to target_pos
        for k, v in action.items():
            if k.endswith(".pos"):
                j_name = k[:-4]
                if j_name in self.motor_map_inv:
                    m_name = self.motor_map_inv[j_name]
                    # Find index
                    if m_name in self.bus.motor_names:
                        idx = self.bus.motor_names.index(m_name)
                        if idx < total_motors:
                            target_pos[idx] = v

        # Update bus buffer for consistency
        self.bus.target_positions = np.array(target_pos)

        # Send explicit command to all motors (including gripper)
        try:
            for i in range(total_motors):
                robot.Motors[i].pos_vel_tqe_kp_kd(
                    target_pos[i],
                    target_vel[i],
                    target_tqe[i],
                    kp[i],
                    kd[i]
                )

            robot.motor_send_cmd()

        except Exception as e:
            logger.error(f"Failed to send action: {e}")

        return action

    def _control_loop(self):
        """Background loop for gravity compensation and state update."""
        logger.info("Starting gravity compensation loop...")

        robot = self.bus.robot
        all_motors = robot.Motors
        total_motors = len(all_motors)

        # Torque limits
        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

        # Position hold gains (based on working PD controller)
        hold_kp = [4.0, 10.0, 10.0, 2.0, 2.0, 1.0, 3.0]  # Last value for gripper
        hold_kd = [0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.3]    # Last value for gripper

        # Ensure arrays have correct length
        if len(hold_kp) < total_motors:
            hold_kp.extend([1.0] * (total_motors - len(hold_kp)))
        if len(hold_kd) < total_motors:
            hold_kd.extend([0.1] * (total_motors - len(hold_kd)))

        # Track position when entering hold mode
        hold_position = None

        while not self._stop_event.is_set():
            try:
                # Only apply if no external action has been sent recently (e.g. > 0.5s)
                if time.time() - self._last_action_time > 0.5:
                    # Get current state
                    current_pos = robot.get_current_pos()
                    vel = robot.get_current_vel()

                    # Capture hold position when first entering hold mode
                    if hold_position is None:
                        hold_position = list(current_pos)
                        if len(hold_position) < total_motors:
                            hold_position.extend([0.0] * (total_motors - len(hold_position)))
                        logger.info("进入位置保持模式，保持当前位置")

                    target_vel = [0.0] * total_motors

                    try:
                        tau_gravity = robot.get_Gravity() # Arm gravity
                        tau_friction = robot.get_friction_compensation(
                            vel=vel,
                            Fc=self.fc,
                            Fv=self.fv,
                            vel_threshold=self.vel_threshold
                        )

                        tau_total_arm = tau_gravity + tau_friction

                        # Clip torque
                        tau_limit = tau_limit_base
                        if len(tau_total_arm) != len(tau_limit):
                             if len(tau_total_arm) > len(tau_limit):
                                  tau_limit = np.append(tau_limit, [5.0] * (len(tau_total_arm) - len(tau_limit)))
                             else:
                                  tau_limit = tau_limit[:len(tau_total_arm)]

                        tau_total_arm = np.clip(tau_total_arm, -tau_limit, tau_limit)

                        # Construct full torque array
                        tau_final = list(tau_total_arm)
                        # Pad for gripper (zero torque)
                        if len(tau_final) < total_motors:
                            tau_final.extend([0.0] * (total_motors - len(tau_final)))

                        # Send position hold command with gravity compensation
                        for i in range(total_motors):
                            robot.Motors[i].pos_vel_tqe_kp_kd(
                                hold_position[i],  # Hold captured position
                                target_vel[i],
                                tau_final[i],      # With gravity compensation
                                hold_kp[i],        # Position gain for stability
                                hold_kd[i]         # Damping for stability
                            )
                        robot.motor_send_cmd()

                    except Exception as e:
                        # logger.warning(f"Control loop calculation error: {e}")
                        pass
                else:
                    # Reset hold position when actively controlled
                    if hold_position is not None:
                        hold_position = None
                        logger.info("退出位置保持模式")

                # Sleep (200Hz)
                time.sleep(0.005)

            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                time.sleep(0.1)

        # Cleanup: zero torque
        try:
            for i in range(total_motors):
                robot.Motors[i].pos_vel_tqe_kp_kd(0, 0, 0, 0, 0)
            robot.motor_send_cmd()
        except:
            pass
        logger.info("Gravity compensation loop stopped.")
