import sys
import tempfile
import ikpy
from ikpy.chain import Chain
import math
from controller import Supervisor

class robot(Supervisor):
    def __init__(self):
        super().__init__()

        self.IKPY_MAX_ITERATIONS = 8
        self.TIME_STEP = 32

        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.getUrdf().encode('utf-8'))

        self.armChain = Chain.from_urdf_file(filename)
        self.armChain.active_links_mask[0] = False
        self.armChain.active_links_mask[6] = False

        #self.target = self.getFromDef('TARGET')
        self.arm = self.getSelf()

        print(self.armChain.links)
        self.motors = []
        timeStep = int(4 * self.getBasicTimeStep())
        for link in self.armChain.links:
            if 'motor' in link.name:
                motor = self.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timeStep)
                self.motors.append(motor)

        self.position_0 = [0] + [0 for m in self.motors] + [0]

    def run_kinematics(self, x, y, z, initial_position, set_pan):
        ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=self.IKPY_MAX_ITERATIONS, initial_position=initial_position)
        # position_fk = self.armChain.forward_kinematics(ikResults)
        # squared_distance = (position_fk[0, 3] - x) ** 2 + (position_fk[1, 3] - y) ** 2 + (position_fk[2, 3] - z) ** 2
        squared_distance = 0

        for i in range(5):
            self.motors[i].setPosition(ikResults[i + 1])

        if (set_pan):
            self.motors[4].setPosition(-(math.pi + ikResults[2] + ikResults[3]))

        return ikResults, squared_distance

    def run(self, x, y, z, set_pan=False, from_zero=False):
        if (set_pan):
            self.armChain.active_links_mask[4] = False
            self.armChain.active_links_mask[5] = False
            end_motor = 4
        else:
            self.armChain.active_links_mask[4] = True
            self.armChain.active_links_mask[5] = True
            end_motor = 6

        if from_zero:
            ikResults, squared_distance = self.run_kinematics(x, y, z, self.position_0, set_pan)
        else:
            position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0]
            ikResults, squared_distance = self.run_kinematics(x, y, z, position, set_pan)

        while self.step(self.TIME_STEP) != -1:
            position = [0] + [m.getPositionSensor().getValue() for m in self.motors] + [0]

            # if not from_zero:
            #     ikResults, squared_distance = self.run_kinematics(x, y, z, position, set_pan)

            pos_err = 0
            for i in range(1, end_motor):
                pos_err += (ikResults[i] - position[i]) ** 2

            if math.sqrt(pos_err) < 0.0001:
                print("end step")
                break

    def draw_circle(self, set_pan=True, from_zero=False):
        print('Draw a circle on the paper sheet...')
        target = self.getFromDef('TABLE_WITH_PAPER_SHEET')
        targetPosition = target.getPosition()
        armPosition = self.arm.getPosition()
        print(targetPosition)
        print(armPosition)
        x0 = targetPosition[0] - armPosition[0]
        y0 = targetPosition[1] - armPosition[1] + 0.08
        z0 = targetPosition[2] - armPosition[2] + 0.17
        print(x0, y0, z0)
        self.getDevice('pen').write(True)
        counter_t = 0

        for counter in range(100):
            x = 0.1 * math.cos(2*math.pi*counter_t/100) + x0
            y = 0.1 * math.sin(2*math.pi*counter_t/100) + y0
            z = z0
            counter_t= counter_t+1
            self.run(x,y,z, set_pan, from_zero)

    def move_to_sphere(self):
        print('Move the yellow and black sphere to move the arm...')
        target = self.getFromDef('TARGET')
        targetPosition = target.getPosition()
        armPosition = self.arm.getPosition()
        print(targetPosition)
        print(armPosition)
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
        print(x, y, z)
        self.run(x,y,z, False, True)

    def draw_line(self, set_pan=True, from_zero=False):
        print('draw_line')
        target = self.getFromDef('TABLE_WITH_PAPER_SHEET')
        targetPosition = target.getPosition()
        armPosition = self.arm.getPosition()
        print(targetPosition)
        print(armPosition)
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2] + 0.17
        print(x, y, z)
        self.getDevice('pen').write(True)
        for dy in range(0, 10, 1):
            self.run(x,y+dy/100,z, set_pan, from_zero)

    def move_to_zero_point(self):
        print('Move arm to zero point...')

        ikResults =  [0] + [0 for m in self.motors] +[0]
        position = self.armChain.forward_kinematics(ikResults)

        x = position[0, 3]
        y = position[1, 3]
        z = position[2, 3]
        print(x, y, z)
        self.run(x,y,z)


robot_obj = robot()
robot_obj.draw_line(True, True)
robot_obj.move_to_zero_point()
robot_obj.draw_circle(True, True)
robot_obj.move_to_sphere()
robot_obj.move_to_zero_point()

