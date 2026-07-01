
from arm_controller import ArmController


arm = ArmController(
    servo_uart_ports=[
        {"servo_id": 0, "port": "COM6"},
        {"servo_id": 1, "port": "COM7"},
        {"servo_id": 2, "port": "COM8"},
    ]
)


def main():
    arm.set_xyz_target(10, 20, 30)
    arm.stop()