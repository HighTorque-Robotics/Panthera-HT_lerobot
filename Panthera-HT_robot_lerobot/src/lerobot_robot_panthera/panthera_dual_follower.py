import numpy as np
import yaml
import os
import sys
import logging
import time
import threading
from functools import cached_property
from dataclasses import dataclass, field
from collections import deque

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
    Configure RealSense cameras automatically:
    - 3 cameras: dual-arm + top view mode (left_wrist + right_wrist + top)
    - 2 D405 cameras: dual-arm mode (left_wrist + right_wrist)
    - 1 D405 camera: single-arm mode (wrist)
    - 1 D405 + 1 D435 camera: single-arm + top view mode (wrist + top)

    Camera serial mapping:
    - D405 (352122273105) -> right wrist
    - D405 (352122272797) -> left wrist
    - D435 (948122071707) -> top view
    """
    try:
        available_cameras = RealSenseCamera.find_cameras()
        cameras = {}

        if available_cameras:
            found_serials = [c["id"] for c in available_cameras]
            logger.info(f"Detected RealSense cameras: {found_serials}")

            d405_right = "352122273105"  # right wrist
            d405_left = "352122272797"   # left wrist
            d435_top = "408322072614"    # top view

            has_right = d405_right in found_serials
            has_left = d405_left in found_serials
            has_top = d435_top in found_serials

            if not has_top:
                raise RuntimeError(
                    f"[ERROR] Error: D435 top-view camera was not detected(serial: {d435_top})\n"
                    f"Detected cameras: {found_serials}\n"
                    f"Dual-arm leader-follower control requires a top-view camera."
                )
            logger.info(f"[OK] Detected D435 top-view camera(serial: {d435_top})")

            camera_map = {}

            if has_left and has_right:

                logger.info("Detected 3 cameras; using dual-arm + top mode")
                camera_map = {
                    d405_right: "right_wrist",
                    d405_left: "left_wrist",
                    d435_top: "top"
                }
            elif has_right or has_left:

                logger.info("Detected 1 D405 and 1 D435 camera; using single-arm + top mode")
                wrist_serial = d405_right if has_right else d405_left
                camera_map = {
                    wrist_serial: "wrist",
                    d435_top: "top"
                }
            else:

                logger.info("Detected only the D435 top-view camera")
                camera_map = {d435_top: "top"}

            for cam_info in available_cameras:
                serial = cam_info["id"]

                if serial in camera_map:
                    key = camera_map[serial]
                    logger.info(f"Configuring camera {key} (serial: {serial})")

                    cameras[key] = RealSenseCameraConfig(
                        serial_number_or_name=serial,
                        fps=30,
                        width=640,
                        height=480,
                        color_mode=ColorMode.RGB,
                        use_depth=False,
                        rotation=Cv2Rotation.NO_ROTATION
                    )

            if cameras:
                return cameras

    except Exception as e:
        logger.error(f"Failed to auto-detect RealSense cameras: {e}")
        raise RuntimeError(
            f"Camera detection failed: {e}\n"
            "Dual-arm leader-follower control requires a D435 top-view camera."
        )

    raise RuntimeError(
        "No RealSense cameras were detected\n"
        "Dual-arm leader-follower control requires a D435 top-view camera."
    )

@RobotConfig.register_subclass("panthera_dual_follower")
@dataclass
class PantheraDualFollowerConfig(RobotConfig):

    left_param_path: str = "robot_param/LeftFollower.yaml"

    right_param_path: str = "robot_param/Follower.yaml"
    cameras: dict[str, CameraConfig] = field(default_factory=default_cameras_factory)

class PantheraDualFollower(Robot):
    """
    Implementation of the Panthera dual-arm follower robot.
    Manages two independent follower arms for coordinated tasks.
    """
    config_class = PantheraDualFollowerConfig
    name = "panthera_dual_follower"

    def __init__(self, config):
        super().__init__(config)
        self.config = config

        def resolve_param_path(param_path):
            """Resolve the parameter file path."""
            if not os.path.isabs(param_path):
                if not os.path.exists(param_path):
                    curr_file = os.path.abspath(__file__)
                    panthera_ht_lerobot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_file)))))
                    panthera_sdk_path = os.path.join(panthera_ht_lerobot, "Panthera-HT_SDK", "panthera_python", param_path)
                    logger.info(f"Looking for config in SDK path: {panthera_sdk_path}")
                    if os.path.exists(panthera_sdk_path):
                        return panthera_sdk_path
                    else:
                        logger.warning(f"Config not found in SDK path: {panthera_sdk_path}")
            return param_path

        left_param_path = resolve_param_path(config.left_param_path)
        right_param_path = resolve_param_path(config.right_param_path)

        logger.info(f"Left arm config: {left_param_path}")
        logger.info(f"Right arm config: {right_param_path}")

        self.left_bus = PantheraMotorsBus(left_param_path)
        self.right_bus = PantheraMotorsBus(right_param_path)

        self.left_joint_names = []
        self.right_joint_names = []

        try:
            with open(left_param_path, 'r', encoding='utf-8') as f:
                left_yaml = yaml.safe_load(f)
                self.left_joint_names = left_yaml.get('kinematics', {}).get('joint_names', [])
        except Exception as e:
            logger.error(f"Failed to load {left_param_path}  left arm config: {e}")

        try:
            with open(right_param_path, 'r', encoding='utf-8') as f:
                right_yaml = yaml.safe_load(f)
                self.right_joint_names = right_yaml.get('kinematics', {}).get('joint_names', [])
        except Exception as e:
            logger.error(f"Failed to load {right_param_path}  right arm config: {e}")

        if len(self.left_joint_names) == 6:
            self.left_joint_names.append("gripper")
            logger.info("Left arm: added gripper to joint_names")

        if len(self.right_joint_names) == 6:
            self.right_joint_names.append("gripper")
            logger.info("Right arm: added gripper to joint_names")

        self.joint_names = []
        for name in self.left_joint_names:
            self.joint_names.append(f"left_{name}")
        for name in self.right_joint_names:
            self.joint_names.append(f"right_{name}")

        logger.info(f"Dual-arm joint names: {self.joint_names}")

        self.cameras = make_cameras_from_configs(config.cameras)

        self.left_motor_map_inv = {}
        self.left_motor_map = {}
        self.right_motor_map_inv = {}
        self.right_motor_map = {}

        self._thread = None
        self._stop_event = threading.Event()
        self._obs_lock = threading.Lock()
        self._latest_motor_pos = None
        self._last_action_time = 0.0

        self.fc = getattr(config, "fc", (0.2, 0.15, 0.15, 0.15, 0.04, 0.04))
        self.fv = getattr(config, "fv", (0.06, 0.06, 0.06, 0.03, 0.02, 0.02))
        self.vel_threshold = getattr(config, "vel_threshold", 0.02)

        self.teaching_mode = False

    @property
    def _motors_ft(self):
        return {f"{name}.pos": float for name in self.joint_names}

    @property
    def _cameras_ft(self):
         features = {}
         for cam_key, cam_config in self.config.cameras.items():
             features[cam_key] = (cam_config.height, cam_config.width, 3)

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
        return (self.left_bus.is_connected and self.right_bus.is_connected and
                all(cam.is_connected for cam in self.cameras.values()))

    def connect(self, calibrate=True):
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} connected")

        # ==================== Connecting left arm ====================
        logger.info("Connecting left arm...")
        self.left_bus.connect()

        for i, name in enumerate(self.left_joint_names):
            if i < len(self.left_bus.motor_names):
                m_name = self.left_bus.motor_names[i]
                self.left_motor_map_inv[name] = m_name
                self.left_motor_map[m_name] = name
                logger.info(f"left arm: Mapped joint '{name}' to motor '{m_name}'")

        try:
            current_pos = self.left_bus.robot.get_current_pos()
            if len(current_pos) == len(self.left_bus.target_positions):
                self.left_bus.target_positions = np.array(current_pos)
        except Exception as e:
            logger.warning(f"Failed to initialize left arm target positions: {e}")

        # ==================== Connecting right arm ====================

        time.sleep(2.0)
        logger.info("Connecting right arm...")
        self.right_bus.connect()

        for i, name in enumerate(self.right_joint_names):
            if i < len(self.right_bus.motor_names):
                m_name = self.right_bus.motor_names[i]
                self.right_motor_map_inv[name] = m_name
                self.right_motor_map[m_name] = name
                logger.info(f"right arm: Mapped joint '{name}' to motor '{m_name}'")

        try:
            current_pos = self.right_bus.robot.get_current_pos()
            if len(current_pos) == len(self.right_bus.target_positions):
                self.right_bus.target_positions = np.array(current_pos)
        except Exception as e:
            logger.warning(f"Failed to initialize right arm target positions: {e}")

        try:
            for cam in self.cameras.values():
                cam.connect()
        except Exception as e:
            logger.error(f"Failed to connect cameras: {e}")
            self.left_bus.disconnect()
            self.right_bus.disconnect()
            raise

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        logger.info(f"{self} connected.")

    def disconnect(self):
        if not self.is_connected:
            return

        if self._thread:
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._thread = None

        self.left_bus.disconnect()
        self.right_bus.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self):

        return True

    def calibrate(self):

        logger.info(f"{self} does not require user-side calibration.")
        pass

    def configure(self):

        pass

    def get_observation(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        obs_dict = {}

        left_motor_vals = self.left_bus.sync_read("Present_Position")
        for m_name, val in left_motor_vals.items():
            if m_name in self.left_motor_map:
                j_name = self.left_motor_map[m_name]
                obs_dict[f"left_{j_name}.pos"] = val

        right_motor_vals = self.right_bus.sync_read("Present_Position")
        for m_name, val in right_motor_vals.items():
            if m_name in self.right_motor_map:
                j_name = self.right_motor_map[m_name]
                obs_dict[f"right_{j_name}.pos"] = val

        for cam_key, cam in self.cameras.items():

            max_retries = 3
            frame = None
            for attempt in range(max_retries):
                try:
                    frame = cam.async_read()
                    break
                except TimeoutError:
                    if attempt == max_retries - 1:
                        logger.error(f"Reading camera {cam_key} timed out after {max_retries} retries.")
                        raise
                    time.sleep(0.005)
                except Exception as e:
                    logger.error(f"Reading camera {cam_key} failed: {e}")
                    raise e

            obs_dict[cam_key] = frame

            if isinstance(cam, RealSenseCamera) and getattr(cam, "use_depth", False):
                try:
                    depth_map = cam.read_depth()
                    if depth_map.ndim == 2:
                        depth_map = np.expand_dims(depth_map, axis=-1)
                    obs_dict[f"{cam_key}_depth"] = depth_map
                except Exception as e:
                    logger.warning(f"Reading depth from camera {cam_key} failed: {e}")

        return obs_dict

    def send_action(self, action):
        self._last_action_time = time.time()

        if self.teaching_mode:

            self._last_action_time = time.time()
            return action

        left_action = {}
        right_action = {}

        for k, v in action.items():
            if k.startswith("left_") and k.endswith(".pos"):

                joint_name = k[5:-4]
                left_action[joint_name] = v
            elif k.startswith("right_") and k.endswith(".pos"):

                joint_name = k[6:-4]
                right_action[joint_name] = v

        self._send_arm_action(self.left_bus, self.left_motor_map_inv, left_action, "left arm")

        self._send_arm_action(self.right_bus, self.right_motor_map_inv, right_action, "right arm")

        return action

    def _send_teaching_mode_command(self, bus, hold_pos=None):
        """Send a teaching-mode command with gravity compensation and low-stiffness position hold.
        hold_pos: Fixed target pose. Use None while dragging; use a captured pose while idle to prevent sagging.
        """
        robot = bus.robot
        all_motors = robot.Motors
        total_motors = len(all_motors)

        try:
            current_pos = robot.get_current_pos()
            tau_gravity = robot.get_Gravity()
            tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

            if len(tau_gravity) > len(tau_limit):
                pad_width = len(tau_gravity) - len(tau_limit)
                tau_limit = np.append(tau_limit, [5.0] * pad_width)

            tau_arm = np.clip(tau_gravity, -tau_limit, tau_limit)

            target_pos = hold_pos if hold_pos is not None else current_pos

            teach_kp = [0.5, 1.0, 1.0, 0.5, 0.3, 0.3]
            teach_kd = [0.05, 0.1, 0.1, 0.05, 0.03, 0.03]
            for i in range(min(len(tau_arm), total_motors)):
                pos = target_pos[i] if i < len(target_pos) else 0.0
                kp_i = teach_kp[i] if i < len(teach_kp) else 0.3
                kd_i = teach_kd[i] if i < len(teach_kd) else 0.03
                robot.Motors[i].pos_vel_tqe_kp_kd(pos, 0.0, tau_arm[i], kp_i, kd_i)

            if total_motors > len(tau_arm):
                gripper_target = 1.6
                for i in range(len(tau_arm), total_motors):
                    robot.Motors[i].pos_vel_tqe_kp_kd(gripper_target, 0.0, 0.0, 0.3, 0.05)

            robot.motor_send_cmd()

        except Exception as e:
            logger.error(f"Teaching-mode gravity compensation failed: {e}")

    def _send_arm_action(self, bus, motor_map_inv, arm_action, arm_name):
        """Send an action command to one arm."""
        robot = bus.robot
        all_motors = robot.Motors
        total_motors = len(all_motors)

        if len(bus.target_positions) < total_motors:
            target_pos = list(bus.target_positions) + [0.0] * (total_motors - len(bus.target_positions))
        else:
            target_pos = list(bus.target_positions)

        target_vel = [0.0] * total_motors

        try:
            tau_gravity = robot.get_Gravity()
            tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

            if len(tau_gravity) > len(tau_limit):
                pad_width = len(tau_gravity) - len(tau_limit)
                tau_limit = np.append(tau_limit, [5.0] * pad_width)
            elif len(tau_gravity) < len(tau_limit):
                tau_limit = tau_limit[:len(tau_gravity)]

            tau_arm = np.clip(tau_gravity, -tau_limit, tau_limit)
            target_tqe = list(tau_arm)
            if len(target_tqe) < total_motors:
                target_tqe.extend([0.0] * (total_motors - len(target_tqe)))
        except Exception as e:
            logger.warning(f"{arm_name} gravity compensation calculation failed: {e}")
            target_tqe = [0.0] * total_motors

        kp_defaults = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0, 4.0]
        kd_defaults = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1, 0.4]

        kp = [0.0] * total_motors
        kd = [0.0] * total_motors

        for i in range(total_motors):
            if i < len(kp_defaults):
                kp[i] = kp_defaults[i]
                kd[i] = kd_defaults[i]
            else:
                kp[i] = 1.0
                kd[i] = 0.1

        for joint_name, pos_value in arm_action.items():
            if joint_name in motor_map_inv:
                m_name = motor_map_inv[joint_name]
                if m_name in bus.motor_names:
                    idx = bus.motor_names.index(m_name)
                    if idx < total_motors:
                        target_pos[idx] = pos_value

        bus.target_positions = np.array(target_pos)

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
            logger.error(f"{arm_name} failed to send action: {e}")

    def _control_loop(self):
        """Background loop for dual-arm gravity compensation and state updates."""
        logger.info("Starting dual-arm gravity compensation loop...")

        left_robot = self.left_bus.robot
        right_robot = self.right_bus.robot

        left_motors = left_robot.Motors
        right_motors = right_robot.Motors

        left_total_motors = len(left_motors)
        right_total_motors = len(right_motors)

        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

        hold_kp = [4.0, 10.0, 10.0, 2.0, 2.0, 1.0, 3.0]
        hold_kd = [0.5, 0.8, 0.8, 0.2, 0.2, 0.1, 0.3]

        left_hold_position = None
        right_hold_position = None
        left_teach_hold_pos = None
        right_teach_hold_pos = None

        while not self._stop_event.is_set():
            try:
                if self.teaching_mode:

                    if left_teach_hold_pos is None:
                        left_teach_hold_pos = list(left_robot.get_current_pos())
                        if len(left_teach_hold_pos) < left_total_motors:
                            left_teach_hold_pos.extend([0.0] * (left_total_motors - len(left_teach_hold_pos)))
                    if right_teach_hold_pos is None:
                        right_teach_hold_pos = list(right_robot.get_current_pos())
                        if len(right_teach_hold_pos) < right_total_motors:
                            right_teach_hold_pos.extend([0.0] * (right_total_motors - len(right_teach_hold_pos)))

                    self._send_teaching_mode_command(self.left_bus, left_teach_hold_pos)
                    self._send_teaching_mode_command(self.right_bus, right_teach_hold_pos)
                    left_hold_position = None
                    right_hold_position = None

                elif time.time() - self._last_action_time > 0.5:

                    if left_hold_position is None:
                        left_hold_position = list(left_robot.get_current_pos())
                        if len(left_hold_position) < left_total_motors:
                            left_hold_position.extend([0.0] * (left_total_motors - len(left_hold_position)))

                    if right_hold_position is None:
                        right_hold_position = list(right_robot.get_current_pos())
                        if len(right_hold_position) < right_total_motors:
                            right_hold_position.extend([0.0] * (right_total_motors - len(right_hold_position)))

                    self._control_single_arm(
                        left_robot, left_total_motors, tau_limit_base,
                        hold_kp, hold_kd, left_hold_position, "left arm"
                    )

                    self._control_single_arm(
                        right_robot, right_total_motors, tau_limit_base,
                        hold_kp, hold_kd, right_hold_position, "right arm"
                    )

                else:

                    if left_hold_position is not None or right_hold_position is not None:
                        left_hold_position = None
                        right_hold_position = None
                        logger.info("Leaving position hold mode")

                time.sleep(0.005)

            except Exception as e:
                logger.error(f"Control loop error: {e}")
                time.sleep(0.1)

        try:
            for i in range(left_total_motors):
                left_robot.Motors[i].pos_vel_tqe_kp_kd(0, 0, 0, 0, 0)
            left_robot.motor_send_cmd()

            for i in range(right_total_motors):
                right_robot.Motors[i].pos_vel_tqe_kp_kd(0, 0, 0, 0, 0)
            right_robot.motor_send_cmd()
        except:
            pass

        logger.info("Dual-arm gravity compensation loop stopped.")

    def _control_single_arm(self, robot, total_motors, tau_limit_base, hold_kp, hold_kd, hold_position, arm_name):
        """Run gravity compensation and position hold for one arm."""
        try:
            current_pos = robot.get_current_pos()
            vel = robot.get_current_vel()

            target_vel = [0.0] * total_motors

            tau_gravity = robot.get_Gravity()
            tau_friction = robot.get_friction_compensation(
                vel=vel,
                Fc=self.fc,
                Fv=self.fv,
                vel_threshold=self.vel_threshold
            )

            tau_total_arm = tau_gravity + tau_friction

            tau_limit = tau_limit_base
            if len(tau_total_arm) != len(tau_limit):
                if len(tau_total_arm) > len(tau_limit):
                    tau_limit = np.append(tau_limit, [5.0] * (len(tau_total_arm) - len(tau_limit)))
                else:
                    tau_limit = tau_limit[:len(tau_total_arm)]

            tau_total_arm = np.clip(tau_total_arm, -tau_limit, tau_limit)

            tau_final = list(tau_total_arm)
            if len(tau_final) < total_motors:
                tau_final.extend([0.0] * (total_motors - len(tau_final)))

            kp = hold_kp[:total_motors] if len(hold_kp) >= total_motors else hold_kp + [1.0] * (total_motors - len(hold_kp))
            kd = hold_kd[:total_motors] if len(hold_kd) >= total_motors else hold_kd + [0.1] * (total_motors - len(hold_kd))

            for i in range(total_motors):
                robot.Motors[i].pos_vel_tqe_kp_kd(
                    hold_position[i],
                    target_vel[i],
                    tau_final[i],
                    kp[i],
                    kd[i]
                )
            robot.motor_send_cmd()

        except Exception as e:
            pass

    def move_to_initial_position(self, duration_s=5.0):
        """
        Move both arms smoothly to the fixed initial position, slightly above zero.
        Use position-velocity mode (pos_vel_MAXtqe).

        Args:
            duration_s: Move duration in seconds; defaults to 5 seconds
        """

        INITIAL_POSITION = [1.57,1.57, 1.57, 0.0, 0.0, 0.0, 1.6]

        logger.info(f"Moving both arms smoothly to the initial position: {INITIAL_POSITION} (duration {duration_s}s)")

        was_teaching_mode = self.teaching_mode
        if was_teaching_mode:
            self.teaching_mode = False
            logger.info("Temporarily disabling teaching mode to move to the initial position")

        was_running = self._thread is not None and self._thread.is_alive()
        if was_running:
            logger.info("Temporarily stopping the control thread to move to the initial position")
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._stop_event.clear()

        left_robot = self.left_bus.robot
        right_robot = self.right_bus.robot

        left_start_pos = left_robot.get_current_pos()
        right_start_pos = right_robot.get_current_pos()

        if len(left_start_pos) < len(INITIAL_POSITION):
            try:
                gripper_pos = left_robot.get_current_pos_gripper()
                left_start_pos = np.append(left_start_pos, gripper_pos)
            except:
                left_start_pos = np.append(left_start_pos, 1.6)

        if len(right_start_pos) < len(INITIAL_POSITION):
            try:
                gripper_pos = right_robot.get_current_pos_gripper()
                right_start_pos = np.append(right_start_pos, gripper_pos)
            except:
                right_start_pos = np.append(right_start_pos, 1.6)

        steps = int(duration_s * 50)  # 50Hz
        left_prev_pos = left_start_pos.copy()
        right_prev_pos = right_start_pos.copy()

        for i in range(steps):
            alpha = (i + 1) / steps
            alpha_smooth = 3 * alpha**2 - 2 * alpha**3

            left_target_pos = [(1 - alpha_smooth) * s + alpha_smooth * t
                              for s, t in zip(left_start_pos, INITIAL_POSITION)]
            right_target_pos = [(1 - alpha_smooth) * s + alpha_smooth * t
                               for s, t in zip(right_start_pos, INITIAL_POSITION)]

            dt = 0.02  # 50Hz
            left_target_vel = [(left_target_pos[j] - left_prev_pos[j]) / dt for j in range(len(left_target_pos))]
            right_target_vel = [(right_target_pos[j] - right_prev_pos[j]) / dt for j in range(len(right_target_pos))]

            left_prev_pos = left_target_pos.copy()
            right_prev_pos = right_target_pos.copy()

            max_torque = [30.0, 30.0, 30.0, 15.0, 5.0, 5.0]

            for j in range(6):
                left_robot.Motors[j].pos_vel_MAXtqe(left_target_pos[j], left_target_vel[j], max_torque[j])

            if len(left_target_pos) > 6:
                left_robot.Motors[6].pos_vel_MAXtqe(left_target_pos[6], left_target_vel[6] if len(left_target_vel) > 6 else 0.0, 5.0)
            left_robot.motor_send_cmd()

            for j in range(6):
                right_robot.Motors[j].pos_vel_MAXtqe(right_target_pos[j], right_target_vel[j], max_torque[j])

            if len(right_target_pos) > 6:
                right_robot.Motors[6].pos_vel_MAXtqe(right_target_pos[6], right_target_vel[6] if len(right_target_vel) > 6 else 0.0, 5.0)
            right_robot.motor_send_cmd()

            time.sleep(0.02)  # 50Hz

        self.left_bus.target_positions = np.array(INITIAL_POSITION[:len(self.left_bus.target_positions)])
        self.right_bus.target_positions = np.array(INITIAL_POSITION[:len(self.right_bus.target_positions)])

        if was_running:
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()

        if was_teaching_mode:
            self.teaching_mode = True
            logger.info("[OK] Both arms reached the initial position; teaching mode re-enabled")
        else:
            logger.info("[OK] Both arms reached the initial position")

    def get_initial_position(self):
        """
        Return the fixed initial position.

        Returns:
            dict: Action mapping for the initial positions of both arms.
        """
        INITIAL_POSITION = [1.57, 1.57, 1.57, 0.0, 0.0, 0.0, 1.6]

        action = {}
        # left arm
        for i, name in enumerate(self.left_joint_names):
            if i < len(INITIAL_POSITION):
                action[f"left_{name}.pos"] = INITIAL_POSITION[i]

        # right arm
        for i, name in enumerate(self.right_joint_names):
            if i < len(INITIAL_POSITION):
                action[f"right_{name}.pos"] = INITIAL_POSITION[i]

        return action

    def enable_teaching_mode(self):
        """
        Enable teaching mode: only gravity compensation is applied, so the robot can be moved manually.
        Used for manual demonstration data collection.
        """
        self.teaching_mode = True
        logger.info("[OK] Teaching mode enabled: the robot can be moved manually")

    def disable_teaching_mode(self):
        """
        Disable teaching mode and restore normal PD control.
        """
        self.teaching_mode = False
        logger.info("[OK] Teaching mode disabled: normal control restored")
