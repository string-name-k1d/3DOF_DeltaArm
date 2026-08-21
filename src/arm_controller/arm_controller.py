

"""Delta arm controller helpers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Mapping, Optional, Sequence, Tuple

import serial

from uservo import UartServoInfo, UartServoManager
from math import sqrt, sin, cos, atan
import numpy as np

@dataclass
class ServoUARTConfig:
    servo_id: int
    port: str
    baudrate: int = 115200
    timeout: float = 0.0
    parity: str = "N"
    stopbits: float = 1
    bytesize: int = 8
    mean_dps: float = 100.0
    is_debug: bool = False
    serial_kwargs: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "ServoUARTConfig":
        data = dict(config)
        serial_kwargs = dict(data.pop("serial_kwargs", {}))
        servo_id = int(data.pop("servo_id"))
        port = str(data.pop("port"))
        baudrate = int(data.pop("baudrate", 115200))
        timeout = float(data.pop("timeout", 0.0))
        parity = str(data.pop("parity", "N"))
        stopbits = data.pop("stopbits", 1)
        bytesize = int(data.pop("bytesize", 8))
        mean_dps = float(data.pop("mean_dps", 100.0))
        is_debug = bool(data.pop("is_debug", False))
        serial_kwargs.update(data)
        return cls(
            servo_id=servo_id,
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            mean_dps=mean_dps,
            is_debug=is_debug,
            serial_kwargs=serial_kwargs,
        )

@dataclass
class ArmMechConfig:
    # Servo arms: linkage for servo to arm
    # front: directly connected to servo
    servo_upper_arm_len: float
    servo_lower_arm_len: float
    
    # offset of servo from center of arm
    servo_pos: Tuple[float, float, float]
    platform_joint_pos: Tuple[float, float, float]
    
    servo_to_arm_base_len: float
    # Arm lengths: linkage for arm to end effector
    upper_arm_len: float
    lower_arm_len: float
    arm_joint_len: float
    
def arm_computer_ik(
    tar_pos: np.ndarray, 
    mech_config: Sequence[ArmMechConfig]
    ) -> Sequence[float]:
    
    results = []
    cur_result = 0.0
    tar_pos = np.array(tar_pos)
    tar_det = np.sum(tar_pos**2)
    
    for config in mech_config:
        """
        Aliases
        """
        B = config.servo_pos
        P = config.platform_joint_pos
        
        base_len = config.servo_to_arm_base_len
        joint_len = config.arm_joint_len
        u_len = config.upper_arm_len
        l_len = config.lower_arm_len
        
        """
        First compute ik of overall arm
        (x, y, z) -> theta (target angle of upper arm)
        """
        E = 0
        F = 0
        G = 0
        t = (-F + sqrt(E**2 + F **2 - G**2)) / (G - E)
        theta = 2 * atan(t)
        
        """
        Then compute ik of servo arm part
        theta -> tar_angle
        """
        # desired point for joint
        p = (base_len + joint_len * cos(theta), joint_len* sin(theta))
        # dist between centers
        dist = sqrt(p[0]**2 + p[1]**2) 
        
        # find intersection of two circles
        a = (u_len**2 + dist**2 - l_len**2) / (2 * dist) # origin to chord dist
        h = sqrt(u_len**2 - a**2) # height of chord
        a /= dist
        h /= dist
        
        # taking solution with larger y coordinates here
        # !!! may need to flip both +- if incorrect !!!
        tar_ctrl_pt = (
            a * p[0] - h * p[1],
            a * p[1] + h * p[0]
            )
        
        # convert control point to angle
        cur_result = atan(tar_ctrl_pt[1] / tar_ctrl_pt[0])
        
        results.append(cur_result)
    
    return results

class ArmController:
    def __init__(
        self,
        servo_uart_ports: Optional[Sequence[Mapping[str, Any] | ServoUARTConfig]] = None,
        ik_solver: Optional[Callable[[float, float, float], Sequence[float] | Mapping[int, float]]] = None,
    ):
        self._servo_configs: Dict[int, ServoUARTConfig] = {}
        self._serial_ports: Dict[int, Any] = {}
        self._servo_managers: Dict[int, Any] = {}
        self._joint_targets: Dict[int, float] = {}
        self.xyz_target: Optional[Tuple[float, float, float]] = None
        self.ik_solver = ik_solver

        if servo_uart_ports:
            self.configure_servo_uart_ports(servo_uart_ports)

    def configure_servo_uart_ports(
        self,
        servo_uart_ports: Sequence[Mapping[str, Any] | ServoUARTConfig],
    ) -> None:
        self.close()
        for config in servo_uart_ports:
            self.add_servo_uart_port(config)

    def add_servo_uart_port(
        self,
        servo_uart_port: Mapping[str, Any] | ServoUARTConfig,
    ) -> Any:
        config = servo_uart_port if isinstance(servo_uart_port, ServoUARTConfig) else ServoUARTConfig.from_mapping(servo_uart_port)

        serial_options = {
            "port": config.port,
            "baudrate": config.baudrate,
            "timeout": config.timeout,
            "parity": config.parity,
            "stopbits": config.stopbits,
            "bytesize": config.bytesize,
        }
        serial_options.update(config.serial_kwargs)
        uart = serial.Serial(**serial_options)
        manager = UartServoManager(uart, is_scan_servo=False, srv_num=1, mean_dps=int(config.mean_dps), is_debug=config.is_debug)
        manager.servos[config.servo_id] = UartServoInfo(config.servo_id)

        self._servo_configs[config.servo_id] = config
        self._serial_ports[config.servo_id] = uart
        self._servo_managers[config.servo_id] = manager
        return manager

    def set_servo_angle(self, servo_id: int, angle: float, **kwargs: Any) -> bool:
        if servo_id not in self._servo_managers:
            raise KeyError(f"Servo {servo_id} has not been configured")

        self._joint_targets[servo_id] = float(angle)
        return bool(self._servo_managers[servo_id].set_servo_angle(servo_id, float(angle), **kwargs))
    
    def get_servo_actual_angle(self, servo_id: int) -> Optional[float]:
        if servo_id not in self._servo_managers:
            raise KeyError(f"Servo {servo_id} has not been configured")
        return self._servo_managers[servo_id].query_servo_angle()
    
    def get_servo_target_angle(self, servo_id: int) -> Optional[float]:
        if servo_id not in self._servo_managers:
            raise KeyError(f"Servo {servo_id} has not been configured")
        return self._joint_targets.get(servo_id)

    def set_joint_targets(self, joint_targets: Mapping[int, float] | Sequence[float]) -> Dict[int, float]:
        if isinstance(joint_targets, Mapping):
            items = joint_targets.items()
        else:
            values = list(joint_targets)
            if len(values) != 3:
                raise ValueError("Delta arm joint targets must contain exactly 3 angles")
            items = enumerate(values)

        for servo_id, angle in items:
            self.set_servo_angle(int(servo_id), float(angle), interval=0)

        return dict(self._joint_targets)

    def set_xyz_target(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        self.xyz_target = (float(x), float(y), float(z))

        if self.ik_solver is not None:
            joint_targets = self.ik_solver(*self.xyz_target)
            self.set_joint_targets(joint_targets)

        return self.xyz_target

    def move_to_position(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        return self.set_xyz_target(x, y, z)

    def stop(self) -> None:
        for servo_id, manager in self._servo_managers.items():
            manager.set_damping(servo_id, power=0)

    def close(self) -> None:
        for uart in self._serial_ports.values():
            try:
                uart.close()
            except Exception:
                pass

        self._serial_ports.clear()
        self._servo_managers.clear()
        self._servo_configs.clear()
        self._joint_targets.clear()

    def __enter__(self) -> "ArmController":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()