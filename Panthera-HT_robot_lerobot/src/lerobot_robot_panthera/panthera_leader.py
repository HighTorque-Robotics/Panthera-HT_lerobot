from dataclasses import dataclass
import os
import yaml
import logging
import time
import threading
import numpy as np
import pinocchio as pin

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from .motors.panthera_motors_bus import PantheraMotorsBus

logger = logging.getLogger(__name__)

@TeleoperatorConfig.register_subclass("panthera_leader")
@dataclass
class PantheraLeaderConfig(TeleoperatorConfig):
    """
    Configuration for the Panthera leader teleoperator.

    Stores the Panthera leader arm configuration parameters.
    Includes the parameter file path and friction compensation settings.
    """

    param_path: str = "robot_param/Leader.yaml"


    fc: tuple = (0.20, 0.15, 0.15, 0.15, 0.04, 0.04)
    fv: tuple = (0.06, 0.06, 0.06, 0.03, 0.02, 0.02)
    vel_threshold: float = 0.02

class PantheraLeader(Teleoperator):
    """
    Implementation of the Panthera leader teleoperator.

    Handles communication, state acquisition, and gravity/friction compensation for the leader arm.
    Main responsibilities:
    1. Connect and disconnect the Panthera motor bus
    2. Read joint positions in real time
    3. Run gravity and friction compensation in a background thread
    4. Manage gripper state when present
    """

    config_class = PantheraLeaderConfig
    name = "panthera_leader"

    def __init__(self, config: PantheraLeaderConfig):
        """
        Initialize the Panthera leader teleoperator.

        Args:
            config: PantheraLeaderConfig instance with the parameter path and compensation settings
        """
        super().__init__(config)
        self.config = config

        param_path = config.param_path
        if not os.path.isabs(param_path):
            if not os.path.exists(param_path):

                curr_file = os.path.abspath(__file__)

                panthera_ht_lerobot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_file)))))

                panthera_sdk_path = os.path.join(panthera_ht_lerobot, "Panthera-HT_SDK", "panthera_python", param_path)
                logger.info(f"Looking for config file: {panthera_sdk_path}")
                if os.path.exists(panthera_sdk_path):
                    param_path = panthera_sdk_path
                    logger.info(f"Found config file: {param_path}")
                else:
                    logger.warning(f"Config file not found: {panthera_sdk_path}")


        self.bus = PantheraMotorsBus(param_path)

        self.joint_names = []
        try:
            with open(param_path, 'r', encoding='utf-8') as f:
                robot_yaml = yaml.safe_load(f)

                self.joint_names = robot_yaml.get('kinematics', {}).get('joint_names', [])
        except Exception as e:
            logger.error(f"Failed to load robot config: {e}")

        if not self.joint_names:

            logger.warning("No joint names found in config; using motor_0..N by default")

        if len(self.joint_names) == 6:
            self.joint_names.append("gripper")
            logger.info("Pre-added gripper to joint_names so action_features include it")

        self.motor_map_inv = {}
        self.motor_map = {}


        self._thread = None
        self._stop_event = threading.Event()
        self._latest_action = {}
        self._action_lock = threading.Lock()


        self.fc = np.array(config.fc)
        self.fv = np.array(config.fv)
        self.vel_threshold = config.vel_threshold

    @property
    def action_features(self):
        """
        Return the action feature definition.

        The teleoperator outputs joint position features.
        Format: {feature_name: dtype}

        Returns:
            dict: Joint position features, for example {"joint1.pos": float, "joint2.pos": float}
        """

        return {f"{name}.pos": float for name in self.joint_names}

    @property
    def feedback_features(self):
        """
        Return the feedback feature definition.

        No force feedback features are implemented in this version.

        Returns:
            dict: Empty mapping because feedback features are not exposed.
        """

        return {}

    @property
    def is_connected(self):
        """
        Return whether the device is connected.

        Returns:
            bool: True if connected, otherwise False.
        """

        return self.bus.is_connected

    @property
    def is_calibrated(self):
        """
        Return whether the device is calibrated.

        Panthera arms are treated as pre-calibrated and do not require user calibration.

        Returns:
            bool: Always True.
        """

        return True

    def connect(self, calibrate=True):
        """
        Connect the Panthera leader, initialize motor mappings, and start gravity compensation.

        This method performs the following steps:
        1. Check connection state to avoid duplicate connections
        2. Connect the motor bus
        3. Build motor-to-joint mappings
        4. Detect and map the gripper motor
        5. Start the gravity compensation background thread

        Args:
            calibrate: Kept for parent API compatibility; currently unused.

        Raises:
            DeviceAlreadyConnectedError: Raised when the device is already connected.
        """

        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected")

        # ============================== Connect the motor bus ==============================
        self.bus.connect()


        self.motor_map_inv = {}
        self.motor_map = {}


        for i, name in enumerate(self.joint_names):
            if i < len(self.bus.motor_names):
                m_name = self.bus.motor_names[i]
                self.motor_map_inv[name] = m_name
                self.motor_map[m_name] = name


        if len(self.bus.htr_motors) > 6:
            gripper_name = "gripper"
            gripper_idx = 6

            if gripper_idx < len(self.bus.motor_names):
                m_name = self.bus.motor_names[gripper_idx]
                logger.info(f"Detected gripper motor {m_name}, mapping it as '{gripper_name}'")

                self.motor_map_inv[gripper_name] = m_name
                self.motor_map[m_name] = gripper_name

        self._stop_event.clear()

        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        logger.info(f"{self} connected and started the gravity compensation thread.")

    def disconnect(self):
        """
        Disconnect the Panthera leader and stop the gravity compensation thread.

        Stops the background thread before disconnecting the motor bus.
        """

        if not self.is_connected:
            return

        try:
            logger.info("Leader Moving to zero position before disconnecting...")
            self.move_to_zero_position(duration_s=5.0)
        except Exception as e:
            logger.warning(f"Leader failed to move to zero position: {e}")

        if self._thread:
            self._stop_event.set()

            self._thread.join(timeout=2.0)
            self._thread = None

        self.bus.disconnect()

    def calibrate(self):
        """
        Calibrate the device (no-op).

        Panthera arms are factory-calibrated and do not require user calibration.
        This method only exists for parent API compatibility.
        """
        logger.info(f"{self} does not require user calibration.")
        pass

    def configure(self):
        """
        Configure the device (no-op).

        Reserved for future options; no parameters are needed in this version.
        """
        pass

    def get_action(self):
        """
        Return the latest joint-position action from the robot.

        This is the main teleoperator interface for joint position data.
        Prefer the background-thread cache; fall back to a synchronous read if needed.

        Returns:
            dict: Joint position mapping, for example {"joint1.pos": 0.1, "joint2.pos": 0.2}.

        Raises:
            DeviceNotConnectedError: Raised when the device is not connected.
        """

        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected")


        with self._action_lock:
            if not self._latest_action:

                try:

                    motor_vals = self.bus.sync_read("Present_Position")
                    action_dict = {}

                    for m_name, val in motor_vals.items():
                        if m_name in self.motor_map:
                            j_name = self.motor_map[m_name]
                            action_dict[f"{j_name}.pos"] = val
                    return action_dict
                except Exception as e:

                    logger.warning(f"Synchronous motor position read failed; returning default values: {e}")
                    return {f"{name}.pos": 0.0 for name in self.joint_names}


            return self._latest_action.copy()

    def send_feedback(self, feedback):
        """
        Send feedback to the device (no-op).

        Force feedback is not implemented; this method is kept for parent API compatibility.

        Args:
            feedback: Feedback mapping, currently unused.
        """
        pass

    def _control_loop(self):
        """
        Background gravity and friction compensation loop.

        Runs in a background thread and performs:
        1. Update motor state at 200 Hz
        2. Compute gravity and friction compensation torques
        3. Send compensation torque commands for passive compliant control
        4. Update the latest joint positions for get_action
        5. Handle gripper compensation and control
        """
        logger.info("Starting gravity compensation loop...")

        robot = self.bus.robot

        num_motors = robot.motor_count
        zero_vel = [0.0] * num_motors

        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])

        while not self._stop_event.is_set():
            try:

                vel = robot.get_current_vel()
                pos = robot.get_current_pos()

                try:
                    gripper_pos = robot.get_current_pos_gripper()
                    pos = np.append(pos, gripper_pos)
                except Exception as e:
                    pass

                action = {}
                for i, name in enumerate(self.joint_names):
                    if i < len(pos):
                        action[f"{name}.pos"] = pos[i]

                with self._action_lock:
                    self._latest_action = action

                try:

                    tau_gravity = robot.get_Gravity()

                    tau_friction = robot.get_friction_compensation(
                        vel=vel,
                        Fc=self.fc,
                        Fv=self.fv,
                        vel_threshold=self.vel_threshold
                    )

                    tau_total = tau_gravity + tau_friction

                    tau_limit = tau_limit_base

                    if len(tau_total) > len(tau_limit):

                        tau_limit = np.append(tau_limit, [5.0] * (len(tau_total) - len(tau_limit)))
                    else:

                        tau_limit = tau_limit[:len(tau_total)]

                    tau_total = np.clip(tau_total, -tau_limit, tau_limit)

                except Exception as e:

                    logger.warning(f"Compensation torque calculation failed; using zero torque: {e}")
                    tau_total = np.zeros(num_motors)

                hold_kp = [5.0, 10.0, 10.0, 8.0, 6.0, 0.5]
                hold_kd = [0.05, 0.1, 0.1, 0.05, 0.03, 0.01]
                for i in range(6):
                    cur_pos = pos[i] if i < len(pos) else 0.0
                    robot.Motors[i].pos_vel_tqe_kp_kd(cur_pos, 0.0, tau_total[i], hold_kp[i], hold_kd[i])

                if num_motors > 6:
                    robot.Motors[6].pos_vel_tqe_kp_kd(1.6, 0.0, 0.0, 0.0, 0.0)

                robot.motor_send_cmd()

                time.sleep(0.005)

            except Exception as e:

                logger.error(f"Control loop error: {e}")

                time.sleep(0.1)


        try:
            zeros = [0.0] * num_motors
            robot.pos_vel_tqe_kp_kd(zeros, zeros, zeros, zeros, zeros)

            robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning(f"Cleanup failed: {e}")

    def move_to_initial_position(self, duration_s=5.0):
        """
        Move smoothly to the fixed initial position, slightly above zero.
        Use position-velocity mode (pos_vel_MAXtqe).

        Args:
            duration_s: Move duration in seconds; defaults to 5 seconds
        """

        INITIAL_POSITION = [1.57, 1.57, 1.57, -1.57, 0.0, 0.0, 1.6]

        logger.info(f"Leader moving smoothly to the initial position: {INITIAL_POSITION} (duration {duration_s}s)")

        was_running = self._thread is not None and self._thread.is_alive()
        if was_running:
            logger.info("Temporarily stopping the gravity compensation thread to move to the initial position")
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
        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
        kp = [20.0, 20.0, 20.0, 10.0, 5.0, 5.0]
        kd = [2.0, 2.0, 2.0, 1.0, 0.5, 0.5]

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
                tau_limit = tau_limit_base[:len(tau_gravity)]
                tau_gravity = np.clip(tau_gravity, -tau_limit, tau_limit)
            except:
                tau_gravity = np.zeros(6)

            for j in range(6):
                tqe = tau_gravity[j] if j < len(tau_gravity) else 0.0
                robot.Motors[j].pos_vel_tqe_kp_kd(target_pos[j], target_vel[j], tqe, kp[j], kd[j])

            if len(target_pos) > 6:
                robot.Motors[6].pos_vel_tqe_kp_kd(target_pos[6], 0.0, 0.0, 0.0, 0.0)

            robot.motor_send_cmd()
            time.sleep(0.02)

        if was_running:
            logger.info("Reached the initial position; restarting the gravity compensation thread")
            self._thread = threading.Thread(target=self._control_loop, daemon=True)
            self._thread.start()
        else:
            logger.info("Reached the initial position")

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

        logger.info(f"Leader moving smoothly to the zero position: {ZERO_POSITION} (duration {duration_s}s)")

        was_running = self._thread is not None and self._thread.is_alive()
        if was_running:
            logger.info("Temporarily stopping the gravity compensation thread to move to the zero position")
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
        tau_limit_base = np.array([15.0, 30.0, 30.0, 15.0, 5.0, 5.0])
        kp = [20.0, 20.0, 20.0, 10.0, 5.0, 5.0]
        kd = [2.0, 2.0, 2.0, 1.0, 0.5, 0.5]

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
                tau_limit = tau_limit_base[:len(tau_gravity)]
                tau_gravity = np.clip(tau_gravity, -tau_limit, tau_limit)
            except:
                tau_gravity = np.zeros(6)

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

        logger.info("Leader reached the zero position")

        logger.info("Gravity compensation loop stopped.")
