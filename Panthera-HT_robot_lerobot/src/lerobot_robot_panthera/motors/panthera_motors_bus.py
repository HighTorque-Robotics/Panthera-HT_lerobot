import time
import numpy as np
import os
import sys
import yaml
import logging
from typing import Optional, Dict, List, Tuple, Any

# Try to import lerobot depende
from lerobot.motors.motors_bus import MotorsBus, MotorCalibration, MotorNormMode

# Try to import hightorque_robot
try:
    import hightorque_robot as htr
except ImportError:
    # Fallback for development if package is not installed in env
    curr_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_dir))))
    if project_root not in sys.path:
        sys.path.append(project_root)
    try:
        import hightorque_robot as htr
    except ImportError:
        print("Warning: hightorque_robot not found")
        htr = None

# Try to import Panthera wrapper
Panthera = None
try:
    # Setup path to import Panthera_lib
    # __file__ is the current file path
    curr_file = os.path.abspath(__file__)
    # /home/sunteng/桌面/Panthera-HT_LeRobot/Panthera-HT_lerobot/Panthera-HT_robot_lerobot/src/lerobot_robot_panthera/motors/panthera_motors_bus.py
    # Go up 6 levels: panthera_motors_bus.py -> motors -> lerobot_robot_panthera -> src -> Panthera-HT_robot_lerobot -> Panthera-HT_lerobot -> Panthera-HT_LeRobot
    panthera_ht_lerobot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(curr_file))))))
    scripts_path = os.path.join(panthera_ht_lerobot, "Panthera-HT_SDK", "panthera_python", "scripts")


    if scripts_path not in sys.path:
        sys.path.append(scripts_path)

    from Panthera_lib.Panthera import Panthera
except ImportError as e:
    print(f"Warning: Failed to import Panthera from {scripts_path}: {e}")
except Exception as e:
    print(f"Warning: Unexpected error importing Panthera: {e}")

logger = logging.getLogger(__name__)

class PantheraMotorsBus(MotorsBus):
    """
    LeRobot MotorsBus implementation for Panthera Robot.
    """
    def __init__(self, config_path, port=None, motors=None):
        # We don't call super().__init__ because it requires specific arguments
        # and we manage our own state.
        # However, we must ensure we have attributes expected by consumers if any.
        
        self.config_path = config_path
        self.robot = None
        self.htr_motors = []
        self.motor_names = []
        self._is_connected = False
        
        # Buffer for writes
        self.target_positions = None
        self.target_velocities = None
        self.target_torques = None
        
        # Mock self.motors to avoid crashes if accessed (though we override usage)
        self.motors = {}
        self.calibration = {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self):
        if self.is_connected:
            return

        if htr is None:
            raise ImportError("hightorque_robot module is required but not found.")

        if not os.path.exists(self.config_path):
             pass

        print(f"Connecting to Panthera Robot with config: {self.config_path}")
        
        if Panthera is None:
             raise ImportError("Panthera class could not be imported.")

        # Panthera class handles config parsing and motor config path resolution internally
        try:
            self.robot = Panthera(self.config_path)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Panthera robot: {e}")

        self.htr_motors = self.robot.get_motors()
        
        if not self.htr_motors:
            raise RuntimeError("No motors found after connection.")
            
        self.motor_names = [f"motor_{i}" for i in range(len(self.htr_motors))]
        
        # Initialize buffers
        num_motors = len(self.htr_motors)
        self.target_positions = np.zeros(num_motors)
        self.target_velocities = np.zeros(num_motors)
        self.target_torques = np.zeros(num_motors)
        
        # Initial read with delay to allow connection stabilization
        time.sleep(1.0)
        self.robot.send_get_motor_state_cmd()
        self.robot.motor_send_cmd()
        
        self._is_connected = True
        logger.info(f"Connected to {num_motors} motors.")

    def disconnect(self):
        if not self.is_connected:
            return
        self.robot = None
        self.htr_motors = []
        self._is_connected = False
        logger.info("Disconnected from Panthera Robot.")

    def read(self, data_name, motor_name, normalize=True):
        # Read single motor (inefficient, use sync_read)
        try:
            motor_idx = self.motor_names.index(motor_name)
        except ValueError:
            raise ValueError(f"Motor {motor_name} not found")

        state = self.htr_motors[motor_idx].get_current_motor_state()
        
        if data_name == "Present_Position":
            return state.position
        elif data_name == "Present_Velocity":
            return state.velocity
        elif data_name == "Present_Torque":
            return state.torque
        else:
            raise ValueError(f"Unknown data_name: {data_name}")

    def write(self, data_name, motor_name, value, normalize=True):
        try:
            motor_idx = self.motor_names.index(motor_name)
        except ValueError:
            raise ValueError(f"Motor {motor_name} not found")
            
        if data_name == "Goal_Position":
            self.target_positions[motor_idx] = value
        elif data_name == "Goal_Velocity":
            self.target_velocities[motor_idx] = value
        elif data_name == "Goal_Torque":
            self.target_torques[motor_idx] = value
        
        pass

    def sync_read(self, data_name):
        """
        Read data from all motors. Returns dict {motor_name: value}.
        """
        self.robot.send_get_motor_state_cmd()
        
        values = {}
        for i, motor in enumerate(self.htr_motors):
            state = motor.get_current_motor_state()
            if data_name == "Present_Position":
                val = state.position
            elif data_name == "Present_Velocity":
                val = state.velocity
            elif data_name == "Present_Torque":
                val = state.torque
            else:
                val = 0.0
            values[self.motor_names[i]] = val
        
        return values

    def sync_write(self, data_name, values):
        """
        Write data to motors. values is dict {motor_name: value}.
        """
        # Update buffers from dict
        for name, val in values.items():
            if name in self.motor_names:
                idx = self.motor_names.index(name)
                if data_name == "Goal_Position":
                    self.target_positions[idx] = val
                elif data_name == "Goal_Velocity":
                    self.target_velocities[idx] = val
                elif data_name == "Goal_Torque":
                    self.target_torques[idx] = val

        # Send commands
        for i in range(len(self.htr_motors)):
            p = self.target_positions[i]
            v = self.target_velocities[i]
            t = self.target_torques[i]
            
            if data_name == "Goal_Position":
                 if t == 0:
                     t = 20.0 # Default max torque

            self.htr_motors[i].pos_vel_MAXtqe(p, v, t)
            
        self.robot.motor_send_cmd()

    # --- Abstract Method Implementations ---

    def _assert_protocol_is_compatible(self, instruction_name: str) -> None:
        pass

    def _handshake(self) -> None:
        pass

    def _find_single_motor(self, motor: str, initial_baudrate: int | None) -> tuple[int, int]:
        # Not applicable
        raise NotImplementedError

    def configure_motors(self) -> None:
        pass

    def disable_torque(self, motors=None, num_retry=0) -> None:
        # Override to avoid super()'s reliance on self.motors
        # For Panthera, we might not need explicit torque disable or SDK handles it
        pass

    def _disable_torque(self, motor: int, model: str, num_retry: int = 0) -> None:
        pass

    def enable_torque(self, motors=None, num_retry=0) -> None:
        # Override to avoid super()'s reliance on self.motors
        pass
        
    @property
    def is_calibrated(self) -> bool:
        return True

    def read_calibration(self):
        return {}

    def write_calibration(self, calibration_dict, cache=True) -> None:
        pass

    def _get_half_turn_homings(self, positions):
        return {}
        
    def _encode_sign(self, data_name, ids_values):
        return ids_values

    def _decode_sign(self, data_name, ids_values):
        return ids_values

    def _split_into_byte_chunks(self, value, length):
        return []

    def broadcast_ping(self, num_retry=0, raise_on_error=False):
        if not self.is_connected:
             return None
        return {i: 0 for i in range(len(self.htr_motors))}
