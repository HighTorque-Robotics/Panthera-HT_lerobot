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
    Configure RealSense cameras automatically:
    - 3 cameras: dual-arm + top view mode (left_wrist + right_wrist + top)
    - 2 D405 cameras: dual-arm mode (left_wrist + right_wrist)
    - 1 D405 camera: single-arm mode (wrist)
    - 1 D405 + 1 D435 camera: single-arm + top view mode (wrist + top)

    Camera serial mapping:
    - D405 (352122273105) -> right wrist
    - D405 (352122272797) -> left wrist
    - D435I (408322072614) -> top view
    - D435 (335222076820) -> side view
    """
    try:
        available_cameras = RealSenseCamera.find_cameras()
        cameras = {}

        if available_cameras:
            found_serials = [c["id"] for c in available_cameras]
            logger.info(f"Found RealSense cameras: {found_serials}")

            d405_right = "352122273105"  # right wrist
            d405_left = "352122272797"   # left wrist
            d435_top = "408322072614"    # top view
            d435_side = "335222076820"   # side view

            has_right = d405_right in found_serials
            has_left = d405_left in found_serials
            has_top = d435_top in found_serials
            has_side = d435_side in found_serials

            camera_map = {}

            if has_left and has_right and has_top and has_side:

                logger.info("Detected 4 cameras; using dual-arm + top + side mode")
                camera_map = {
                    d405_right: "right_wrist",
                    d405_left: "left_wrist",
                    d435_top: "top",
                    d435_side: "side"
                }
            elif has_left and has_right and has_top:

                logger.info("Detected 3 cameras; using dual-arm + top mode")
                camera_map = {
                    d405_right: "right_wrist",
                    d405_left: "left_wrist",
                    d435_top: "top"
                }
            elif has_left and has_right and has_side:

                logger.info("Detected 3 cameras; using dual-arm + side mode")
                camera_map = {
                    d405_right: "right_wrist",
                    d405_left: "left_wrist",
                    d435_side: "side"
                }
            elif has_left and has_right:

                logger.info("Detected 2 D405 cameras; using dual-arm mode")
                camera_map = {
                    d405_right: "right_wrist",
                    d405_left: "left_wrist"
                }
            elif (has_right or has_left) and has_top and has_side:

                logger.info("Detected 1 D405, 1 D435I, and 1 D435 camera; using single-arm + top + side mode")
                wrist_serial = d405_right if has_right else d405_left
                camera_map = {
                    wrist_serial: "wrist",
                    d435_top: "top",
                    d435_side: "side"
                }
            elif (has_right or has_left) and has_top:

                logger.info("Detected 1 D405 and 1 D435 camera; using single-arm + top mode")
                wrist_serial = d405_right if has_right else d405_left
                camera_map = {
                    wrist_serial: "wrist",
                    d435_top: "top"
                }
            elif (has_right or has_left) and has_side:

                logger.info("Detected 1 D405 and 1 D435 camera; using single-arm + side mode")
                wrist_serial = d405_right if has_right else d405_left
                camera_map = {
                    wrist_serial: "wrist",
                    d435_side: "side"
                }
            elif has_right or has_left:

                logger.info("Detected 1 D405 camera; using single-arm mode")
                wrist_serial = d405_right if has_right else d405_left
                camera_map = {wrist_serial: "wrist"}
            elif has_top:

                logger.info("Detected only the D435 top-view camera")
                camera_map = {d435_top: "top"}
            else:

                logger.warning("No known camera serials detected; using the first available camera")
                if found_serials:
                    camera_map = {found_serials[0]: "wrist"}

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
        logger.warning(f"Failed to auto-detect RealSense cameras: {e}")

    logger.info("Using default single-arm camera configuration")
    return {
        "wrist": RealSenseCameraConfig(
            serial_number_or_name="352122273105",
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

        if len(self.joint_names) == 6:
            self.joint_names.append("gripper")
            logger.info("Pre-added gripper to joint_names so action_features include it")

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

        self.teaching_mode = False

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

        for i, name in enumerate(self.joint_names):
            if i < len(self.bus.motor_names):
                m_name = self.bus.motor_names[i]
                self.motor_map_inv[name] = m_name
                self.motor_map[m_name] = name
                logger.info(f"Mapped joint '{name}' to motor '{m_name}'")

        if "gripper" in self.joint_names:
            if "gripper" in self.motor_map_inv:
                logger.info(f"Gripper mapped successfully: gripper -> {self.motor_map_inv['gripper']}")
            else:
                logger.warning("Gripper exists in joint_names but could not be mapped to a motor")

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

        try:
            logger.info("Moving to zero position before disconnecting...")
            self.move_to_zero_position(duration_s=5.0)
        except Exception as e:
            logger.warning(f"Failed to move to zero position: {e}")

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

        if self.teaching_mode:
            robot = self.bus.robot
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

                for i in range(min(len(tau_arm), total_motors)):
                    pos = current_pos[i] if i < len(current_pos) else 0.0
                    robot.Motors[i].pos_vel_tqe_kp_kd(
                        pos,
                        0.0,
                        tau_arm[i],
                        0.0,
                        0.0
                    )

                if total_motors > len(tau_arm):

                    gripper_target = 1.6
                    for i in range(len(tau_arm), total_motors):
                        robot.Motors[i].pos_vel_tqe_kp_kd(
                            gripper_target,
                            0.0, 0.0, 0.3, 0.05
                        )

                robot.motor_send_cmd()

            except Exception as e:
                logger.error(f"Teaching-mode gravity compensation failed: {e}")

            return action

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

        # kp_defaults = [10.0, 21.0, 21.0, 16.0, 13.0, 1.0, 4.0]
        # kd_defaults = [1.0, 2.0, 2.0, 0.9, 0.8, 0.1, 0.4]
        # Increased by ~10-15% for slightly more force:
        kp_defaults = [11.0, 23.0, 23.0, 18.0, 15.0, 1.2, 4.5]
        kd_defaults = [1.1, 2.2, 2.2, 1.0, 0.9, 0.12, 0.45]

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
                        logger.info("Entering position hold mode and holding the current pose")

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
                        logger.info("Leaving position hold mode")

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

    def move_to_initial_position(self, duration_s=5.0):
        """
        Move smoothly to the fixed initial position, slightly above zero.
        Use position-velocity mode (pos_vel_MAXtqe).

        Args:
            duration_s: Move duration in seconds; defaults to 5 seconds
        """

        INITIAL_POSITION = [1.57, 1.57, 1.57, -1.57, 0.0, 0.0, 1.6]

        logger.info(f"Moving smoothly to the initial position: {INITIAL_POSITION} (duration {duration_s}s)")

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

        robot = self.bus.robot
        start_pos = robot.get_current_pos()

        if len(start_pos) < len(INITIAL_POSITION):
            try:
                gripper_pos = robot.get_current_pos_gripper()
                start_pos = np.append(start_pos, gripper_pos)
            except:
                start_pos = np.append(start_pos, 1.6)

        steps = int(duration_s * 50)
        prev_pos = start_pos.copy()

        for i in range(steps):
            alpha = (i + 1) / steps
            alpha_smooth = 3 * alpha**2 - 2 * alpha**3

            target_pos = [(1 - alpha_smooth) * s + alpha_smooth * t
                         for s, t in zip(start_pos, INITIAL_POSITION)]

            dt = 0.02
            target_vel = [(target_pos[j] - prev_pos[j]) / dt for j in range(len(target_pos))]
            prev_pos = target_pos.copy()

            try:
                tau_gravity = robot.get_Gravity()
                tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])[:len(tau_gravity)]
                tau_gravity = np.clip(tau_gravity, -tau_limit, tau_limit)
            except:
                tau_gravity = np.zeros(6)

            kp = [20.0, 20.0, 20.0, 10.0, 5.0, 5.0]
            kd = [2.0, 2.0, 2.0, 1.0, 0.5, 0.5]
            for j in range(6):
                tqe = tau_gravity[j] if j < len(tau_gravity) else 0.0
                robot.Motors[j].pos_vel_tqe_kp_kd(target_pos[j], target_vel[j], tqe, kp[j], kd[j])

            if len(target_pos) > 6:
                robot.Motors[6].pos_vel_tqe_kp_kd(target_pos[6], 0.0, 0.0, 4.0, 0.4)

            robot.motor_send_cmd()
            time.sleep(0.02)

        self.bus.target_positions = np.array(INITIAL_POSITION[:len(self.bus.target_positions)])

        if was_running:
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()

        if was_teaching_mode:
            self.teaching_mode = True
            logger.info("[OK] Reached the initial position; teaching mode re-enabled")
        else:
            logger.info("[OK] Reached the initial position")

    def get_initial_position(self):
        """
        Return the fixed initial position.

        Returns:
            dict: Action mapping for the initial position.
        """
        INITIAL_POSITION = [1.57, 1.57, 1.57, -1.57, 0.0, 0.0, 1.6]

        action = {}
        for i, name in enumerate(self.joint_names):
            if i < len(INITIAL_POSITION):
                action[f"{name}.pos"] = INITIAL_POSITION[i]

        return action

    def move_to_zero_position(self, duration_s=5.0):
        """
        Move smoothly to the zero position with all joints at zero.

        Args:
            duration_s: Move duration in seconds; defaults to 5 seconds
        """
        ZERO_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6]

        logger.info(f"Moving smoothly to the zero position: {ZERO_POSITION} (duration {duration_s}s)")

        was_teaching_mode = self.teaching_mode
        if was_teaching_mode:
            self.teaching_mode = False
            logger.info("Temporarily disabling teaching mode to move to the zero position")

        was_running = self._thread is not None and self._thread.is_alive()
        if was_running:
            logger.info("Temporarily stopping the control thread to move to the zero position")
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._stop_event.clear()

        robot = self.bus.robot
        start_pos = robot.get_current_pos()

        if len(start_pos) < len(ZERO_POSITION):
            try:
                gripper_pos = robot.get_current_pos_gripper()
                start_pos = np.append(start_pos, gripper_pos)
            except:
                start_pos = np.append(start_pos, 1.6)

        steps = int(duration_s * 50)
        prev_pos = start_pos.copy()

        for i in range(steps):
            alpha = (i + 1) / steps
            alpha_smooth = 3 * alpha**2 - 2 * alpha**3

            target_pos = [(1 - alpha_smooth) * s + alpha_smooth * t
                         for s, t in zip(start_pos, ZERO_POSITION)]

            dt = 0.02
            target_vel = [(target_pos[j] - prev_pos[j]) / dt for j in range(len(target_pos))]
            prev_pos = target_pos.copy()

            try:
                tau_gravity = robot.get_Gravity()
                tau_limit = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])[:len(tau_gravity)]
                tau_gravity = np.clip(tau_gravity, -tau_limit, tau_limit)
            except:
                tau_gravity = np.zeros(6)

            kp = [20.0, 20.0, 20.0, 10.0, 5.0, 5.0]
            kd = [2.0, 2.0, 2.0, 1.0, 0.5, 0.5]
            for j in range(6):
                tqe = tau_gravity[j] if j < len(tau_gravity) else 0.0
                robot.Motors[j].pos_vel_tqe_kp_kd(target_pos[j], target_vel[j], tqe, kp[j], kd[j])

            if len(target_pos) > 6:
                robot.Motors[6].pos_vel_tqe_kp_kd(target_pos[6], 0.0, 0.0, 4.0, 0.4)

            robot.motor_send_cmd()
            time.sleep(0.02)

        self.bus.target_positions = np.array(ZERO_POSITION[:len(self.bus.target_positions)])

        if was_running:
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()

        if was_teaching_mode:
            self.teaching_mode = True
            logger.info("[OK] Reached the zero position; teaching mode re-enabled")
        else:
            logger.info("[OK] Reached the zero position")

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

        logger.info("Gravity compensation loop stopped.")
