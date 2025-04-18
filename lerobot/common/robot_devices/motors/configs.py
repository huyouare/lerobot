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

import abc
from dataclasses import dataclass, field

import draccus


@dataclass
class MotorsBusConfig(draccus.ChoiceRegistry, abc.ABC):
    @property
    def type(self) -> str:
        return self.get_choice_name(self.__class__)


@MotorsBusConfig.register_subclass("dynamixel")
@dataclass
class DynamixelMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("feetech")
@dataclass
class FeetechMotorsBusConfig(MotorsBusConfig):
    port: str
    motors: dict[str, tuple[int, str]]
    mock: bool = False


@MotorsBusConfig.register_subclass("piper")
@dataclass
class PiperMotorsBusConfig(MotorsBusConfig):
    """Configuration for the PiPER arm's motor bus (CAN interface)."""
    can_port: str = "can0"  # default CAN interface name for PiPER
    # Map joint names to an identifier (we use index numbers as IDs for PiPER joints):
    motors: dict[str, tuple[int, str]] = field(default_factory=lambda: {
        "base":    (1, "piper_joint"),
        "shoulder":(2, "piper_joint"),
        "elbow":   (3, "piper_joint"),
        "wrist_pitch": (4, "piper_joint"),
        "wrist_yaw":   (5, "piper_joint"),
        "wrist_roll":  (6, "piper_joint"),
        # "gripper": (7, "piper_gripper")  # if a gripper is attached, optional
    })
    mock: bool = False  # allow simulation mode if needed
