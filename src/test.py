
from arm_controller.arm_controller import ArmController


arm = ArmController()


def main():
    # Example usage of the ArmController
    arm.move_to_position(10, 20, 30)
    arm.stop()