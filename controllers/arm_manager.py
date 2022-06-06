from controller import Supervisor
import math
from arm_kinematics import Kinematics
import tempfile
from controller import Camera

class Robot(Supervisor):
    def __init__(self):
        super().__init__()

        self.TIME_STEP = 32
        self.arm = self.getSelf()
        self.create_motors()
        self.create_kinematics()
        self.cam = self.getDevice('camera')
        self.cam.enable(100)
        # self.disensor = self.getDevice('disensor')
        # self.disensor.enable(100)

        self.home_positions = [0, 0, 0, 0, 0]

    def create_kinematics(self):
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.getUrdf().encode('utf-8'))
        self.obj_kinematics = Kinematics(filename)

    def create_motors(self):
        timeStep = int(4 * self.getBasicTimeStep())
        self.motors = []
        for i in range(1, 6):
            motor = self.getDevice('motor' + str(i))
            motor.setVelocity(1.0)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            self.motors.append(motor)

    def runTo_ik(self, coordinates, initial_position=None):
        if initial_position is None:
            initial_position = [0] + [0 for m in self.motors] + [0,0]

        positions = self.obj_kinematics.run_inverse_kinematics_positions(coordinates, initial_position)

        return self.runTo(positions[1:6])

    def runTo(self, positions):
        ikpy_positions = [0] + [p for p in positions] + [0, 0]
        res = self.obj_kinematics.run_forward_kinematics(ikpy_positions)

        for i in range(0, len(positions)):
            self.motors[i].setPosition(positions[i])
        while self.step(self.TIME_STEP) != -1:
            sensor_position = [m.getPositionSensor().getValue() for m in self.motors]
            pos_err = 0
            for i in range(0, len(positions)):
                pos_err += (positions[i] - sensor_position[i]) ** 2

            if math.sqrt(pos_err) < 0.0001:
                break

        return res

    def home(self):
        return self.runTo(self.home_positions)

if __name__ == "__main__":
    obj_robot = Robot()

    print("Home")
    res = obj_robot.home()
    print(res)

    # positions = [0, math.pi/2, 0, 0, 0]
    # print("Position: ", positions)
    # res = obj_robot.runTo(positions)
    p  = 0
    while p<300:
        positions = [p, -math.pi/3, math.pi/2, 0, math.pi/2]
        res = obj_robot.runTo(positions)
        p = p - math.pi / 36
    print(res)

    coordinate = [res[0, 3], res[1, 3], res[2, 3]]
    res = obj_robot.runTo_ik(coordinate)
    print(res)
