import math
from arm_manager import Robot

class Gripper(Robot):
    def __init__(self):
        super().__init__()
        self.pick_positions = [0, math.pi / 4, -5 * math.pi / 6, 0, math.pi / 8]

        self.create_gripper_motors()

    def create_gripper_motors(self):
        timeStep = int(4 * self.getBasicTimeStep())
        self.gripper = []
        for i in range(1, 3):
            motor = self.getDevice('motor_g' + str(i))
            motor.setVelocity(1.0)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            self.gripper.append(motor)

    def close_gripper(self):
        self.gripper[0].setPosition(-0.04)
        self.gripper[1].setPosition(0.04)
        if self.step(1500) == -1:
            return

    def open_gripper(self):
        self.gripper[0].setPosition(0)
        self.gripper[1].setPosition(0)
        if self.step(1500) == -1:
            return

    def pick(self):
        self.runTo(self.pick_positions)
        self.close_gripper()
        self.home()
        self.open_gripper()

if __name__ == "__main__":
    robot_obj = Gripper()
    robot_obj.pick()


