# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import traceback
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from lerobot.common.robot_devices.motors.configs import PiperMotorsBusConfig
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.utils.utils import capture_timestamp_utc
from lerobot.common.robot_devices.motors.utils import MotorsBus

try:
    from piper_control import piper_control
except ImportError:
    print("Warning: piper_control package not found. Install it to use the Piper arm.")
    piper_control = None


class PiperMotorsBus(MotorsBus):
    """Motor bus implementation for the PiPER robotic arm using CAN interface.
    
    Note: Before using the arm, you need to configure the CAN interface:
        sudo ip link set can0 type can bitrate 1000000
        sudo ip link set up can0
    """

    def __init__(self, config: PiperMotorsBusConfig):
        self.config = config
        self.motors = config.motors
        self.mock = config.mock
        self._piper = None                   # will hold PiperControl instance
        self.is_connected = False
        self.logs = {}

        if not self.mock and piper_control is None:
            raise ImportError("piper_control package not found. Install it to use the Piper arm.")

    def connect(self):
        """Connect to the PiPER arm via CAN and enable motion control."""
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                f"PiPER arm is already connected. Do not call `motors_bus.connect()` twice."
            )

        if self.mock:
            # In mock mode, we don't need the actual piper_control package
            self._piper = type('MockPiperControl', (), {
                'get_joint_positions': lambda: [0.0] * 6,
                'set_joint_positions': lambda x: None,
                'reset': lambda: None,
                'disable': lambda: None
            })()
        else:
            if piper_control is None:
                raise ImportError("piper_control package not found. Install it to use the Piper arm.")
            
            # Initialize the PiperControl interface for the given CAN port
            print(f"Initializing PiPER control on {self.config.can_port}...")
            self._piper = piper_control.PiperControl(can_port=self.config.can_port)
            
            # Get current positions before enabling motion control
            print("Getting current positions...")
            current_positions = self._piper.get_joint_positions()
            print(f"Current positions: {current_positions}")
            
            # Enable motion control
            print("Enabling motion control...")
            self._piper.enable()
            print("Motion control enabled")
            
            # Set the arm back to its current position
            print("Setting arm to current positions...")
            self._piper.set_joint_positions(current_positions)
            print("Arm position restored")
            
        self.is_connected = True
        print(f"Connected to PiPER on {self.config.can_port}")

    def disconnect(self):
        """Clean up the connection to the PiPER arm without disabling it."""
        # Just clean up our connection state
        self.is_connected = False
        self._piper = None
        print("PiPER connection cleaned up.")

    def get_joint_positions(self):
        """Read all joint angles from the PiPER arm."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"PiPER arm not connected. You need to run `motors_bus.connect()`."
            )

        start_time = time.perf_counter()
        
        # Get positions from the arm
        positions = self._piper.get_joint_positions()  # returns a list of 6 floats
        
        # Log timing information
        delta_ts_name = "delta_timestamp_s_read_joint_positions"
        self.logs[delta_ts_name] = time.perf_counter() - start_time
        
        ts_utc_name = "timestamp_utc_read_joint_positions"
        self.logs[ts_utc_name] = capture_timestamp_utc()
        
        return positions

    def set_joint_positions(self, targets):
        """Set multiple joint angles on the PiPER arm."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"PiPER arm not connected. You need to run `motors_bus.connect()`."
            )

        start_time = time.perf_counter()
        
        # If targets is provided as a dict of joint names -> angles, convert to list:
        if isinstance(targets, dict):
            # start from current positions and update given joints
            current = self._piper.get_joint_positions()
            for name, angle in targets.items():
                idx = self.motors[name][0] - 1  # convert 1-indexed ID to 0-index
                current[idx] = angle
            targets_list = current
        else:
            targets_list = list(targets)
            
        # Command the new joint positions
        self._piper.set_joint_positions(targets_list)
        
        # Log timing information
        delta_ts_name = "delta_timestamp_s_write_joint_positions"
        self.logs[delta_ts_name] = time.perf_counter() - start_time
        
        ts_utc_name = "timestamp_utc_write_joint_positions"
        self.logs[ts_utc_name] = capture_timestamp_utc()
    
    def set_joint_position(self, joint_name, angle):
        """Convenience: set a single joint by name."""
        self.set_joint_positions({joint_name: angle})
    
    def open_gripper(self, value=1.0):
        """Open/close gripper if attached (value range 0.0 to 1.0 for fully open)."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"PiPER arm not connected. You need to run `motors_bus.connect()`."
            )
            
        try:
            # Assume PiperControl has a method to control gripper
            self._piper.set_gripper(value)  
        except AttributeError:
            print("Gripper control not supported by PiperControl or no gripper present.")

    def motor_names(self) -> List[str]:
        return list(self.motors.keys())

    def set_calibration(self):
        """Set calibration for the motors."""
        # Implement calibration if needed
        pass

    def apply_calibration(self):
        """Apply calibration to the motors."""
        # Implement calibration application if needed
        pass

    def revert_calibration(self):
        """Revert calibration for the motors."""
        # Implement calibration reversion if needed
        pass

    def read(self) -> Dict[str, float]:
        """Read current positions of all motors."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"PiPER arm not connected. You need to run `motors_bus.connect()`."
            )
            
        positions = self.get_joint_positions()
        print(f"Read positions: {positions}")  # Debug print
        return {name: positions[self.motors[name][0] - 1] for name in self.motors.keys()}

    def write(self, positions: Dict[str, float]):
        """Write target positions to the motors."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                f"PiPER arm not connected. You need to run `motors_bus.connect()`."
            )
            
        # Convert positions dict to a list in the correct order
        target_positions = [0.0] * len(self.motors)
        for name, angle in positions.items():
            idx = self.motors[name][0] - 1  # convert 1-indexed ID to 0-index
            target_positions[idx] = angle
            
        # Send the position command
        print(f"Sending positions to arm: {target_positions}")  # Debug print
        self._piper.set_joint_positions(target_positions)
        print("Positions sent")  # Debug print

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect() 