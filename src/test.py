
from arm_controller import ArmController
import sys
sys.path.append("../../src")
import time

UART_SERVO_PORT="COM7"

arm = ArmController(
    servo_uart_ports=[
        {"servo_id": 0, "port": UART_SERVO_PORT},
        {"servo_id": 1, "port": UART_SERVO_PORT},
        {"servo_id": 2, "port": UART_SERVO_PORT},
    ]
)


def main():
    while (True) {
        print("当前舵机角度: {:4.1f} °".format(angle), end='\r')
        time.sleep(0.5)
    }
    # arm.set_xyz_target(10, 20, 30)
    # arm.stop()
    
main()