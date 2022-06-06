import sys
import tempfile
try:
    import ikpy
    from ikpy.chain import Chain
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

def run(x,y,z,TH):
    #print("RUN")
    #print(TH, x, y, z)
    squared_distance=0
    position = []
    position1 = []
    counter = 0
    while supervisor.step(TIME_STEP) != -1:
        position1 = [0] + [m.getPositionSensor().getValue() for m in motors] +[0]
        initial_position = position1#[0] + [0 for m in motors] +[0]
        #print(initial_position)
        ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)
        
        #print(ikResults)
        #print(position1)
        err0=0
        for i in range(1,6):
            err0 += (ikResults[i]-position1[i])**2
        
        #print(math.sqrt(err0))
        
        for i in range(5):
            motors[i].setPosition(ikResults[i+1])
            
        #motors[4].setPosition(-(math.pi + ikResults[2] + ikResults[3]))
        # Keep the hand orientation perpendicular.
        #motors[5].setPosition(ikResults[1])

        position = armChain.forward_kinematics(ikResults)
        squared_distance = (position[0, 3] - x)**2 + (position[1, 3] - y)**2 + (position[2, 3] - z)**2
        #print(math.sqrt(squared_distance))
        #print(counter)
        if math.sqrt(squared_distance) > 0.0001:
            print("DISTANCE TOO HIGH")
            print(counter, x, y, z)
            print(initial_position)
            print(position[0,3], position[1,3],position[2,3])
            print(ikResults)
            print(math.sqrt(squared_distance))
        
        counter = counter+1
        if math.sqrt(err0) < 0.0001:
            break
    
    
    #for i in range(5):
    #    tmp = [m.getPositionSensor().getValue() for m in motors]
    #print("--------------")
    #print(TH, squared_distance, x, y, z, position[0,3], position[1,3],position[2,3])        
    return squared_distance



IKPY_MAX_ITERATIONS = 4
TIME_STEP = 32

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))

armChain = Chain.from_urdf_file(filename)

armChain.active_links_mask[0] = False
#armChain.active_links_mask[4] = False
#armChain.active_links_mask[5] = False
armChain.active_links_mask[6] = False

print(armChain.links)
print(armChain.active_links_mask)
# Initialize the arm motors and encoders.
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)
        print(link.name)


# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()


print('Draw a circle on the paper sheet...')
counter_t = 0
#for t in range(100):
#while supervisor.step(TIME_STEP) != -1:
for counter in range(100):
    t = supervisor.getTime()
    x = 0.25 * math.cos(2*math.pi*counter_t/100) - 0
    y = 0.25 * math.sin(2*math.pi*counter_t/100) - 0
    z = 0.125
    counter_t= counter_t+1
    err = run(x,y,z,50)
    #print("err: ", str(err))


if 1:
    print('Move the yellow and black sphere to move the arm...')
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()
    print(targetPosition)
    print(armPosition)
    x = targetPosition[0] - armPosition[0]
    y = targetPosition[1] - armPosition[1]
    z = targetPosition[2] - armPosition[2]
    print(x, y, z)
    run(x,y,z,40)


    print('Move arm to zero point...')

    ikResults =  [0] + [0 for m in motors] +[0]
    position = armChain.forward_kinematics(ikResults)

    x = position[0, 3]
    y = position[1, 3]
    z = position[2, 3]
    print(x, y, z)
    run(x,y,z,40)










import sys
import tempfile
import ikpy
from ikpy.chain import Chain
import math
from controller import Supervisor

class robot(Supervisor):
    def __init__(self):
        super().__init__()

        self.IKPY_MAX_ITERATIONS = 4
        self.TIME_STEP = 32

        # Initialize the Webots Supervisor.
        supervisor = Supervisor()
        #timeStep = int(4 * supervisor.getBasicTimeStep())
        # Create the arm chain from the URDF
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.getUrdf().encode('utf-8'))

        self.armChain = Chain.from_urdf_file(filename)
        self.armChain.active_links_mask[0] = False
        # armChain.active_links_mask[4] = False
        # armChain.active_links_mask[5] = False
        self.armChain.active_links_mask[6] = False

        self.target = self.getFromDef('TARGET')
        self.arm = self.getSelf()

        print(self.armChain.links)
        # Initialize the arm motors and encoders.
        self.motors = []
        timeStep = int(4 * self.getBasicTimeStep())
        for link in self.armChain.links:
            if 'motor' in link.name:
                motor = self.getDevice(link.name)
                motor.setVelocity(1.0)
                position_sensor = motor.getPositionSensor()
                position_sensor.enable(timeStep)
                self.motors.append(motor)
                print(link.name)

        self.position_0 = [0] + [0 for m in self.motors] + [0]

    def run_kinematics(x, y, z, initial_position, set_pan):
        ikResults = self.armChain.inverse_kinematics([x, y, z], max_iter=self.IKPY_MAX_ITERATIONS, initial_position=initial_position)
        position_fk = self.armChain.forward_kinematics(ikResults)
        squared_distance = (position_fk[0, 3] - x) ** 2 + (position_fk[1, 3] - y) ** 2 + (position_fk[2, 3] - z) ** 2

        for i in range(5):
            motors[i].setPosition(ikResults[i + 1])

        if (set_pan):
            motors[4].setPosition(-(math.pi + ikResults[2] + ikResults[3]))
            motors[5].setPosition(ikResults[1])

        return ikResults, squared_distance

    def run(x, y, z, set_pan=False, from_zero=False):
        if (from_zero):
            self.armChain.active_links_mask[4] = False
            self.armChain.active_links_mask[5] = False

        ikResults, squared_distance = run_kinematics(x, y, z, self.position_0, set_pan)

        while self.step(TIME_STEP) != -1:
            position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
            if(from_zero):
                ikResults, squared_distance = run_kinematics(x, y, z, position, set_pan)

            pos_err = 0
            for i in range(1, 6):
                pos_err += (ikResults[i] - position[i]) ** 2

            if math.sqrt(squared_distance) > 0.0001 or  math.sqrt(pos_err) < 0.0001:
                print("DISTANCE TOO HIGH")
                print(x, y, z)

    def draw_circle(self, set_pan=False, from_zero=False):
        print('Draw a circle on the paper sheet...')
        counter_t = 0
        for counter in range(100):
            t = supervisor.getTime()
            x = 0.25 * math.cos(2*math.pi*counter_t/100) - 0
            y = 0.25 * math.sin(2*math.pi*counter_t/100) - 0
            z = 0.125
            counter_t= counter_t+1
            run(x,y,z, set_pan, from_zero)

    def move_to_sphere(self):
        print('Move the yellow and black sphere to move the arm...')
        targetPosition = self.target.getPosition()
        armPosition = self.arm.getPosition()
        print(targetPosition)
        print(armPosition)
        x = targetPosition[0] - armPosition[0]
        y = targetPosition[1] - armPosition[1]
        z = targetPosition[2] - armPosition[2]
        print(x, y, z)
        run(x,y,z)

    def move_to_zero_point(self):
        print('Move arm to zero point...')

        ikResults =  [0] + [0 for m in motors] +[0]
        position = self.armChain.forward_kinematics(ikResults)

        x = position[0, 3]
        y = position[1, 3]
        z = position[2, 3]
        print(x, y, z)
        run(x,y,z)


robot_obj = robot()
#robot_obj.draw_circle()
robot_obj.move_to_sphere()
#robot_obj.move_to_zero_point()