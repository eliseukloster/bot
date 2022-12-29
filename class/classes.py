#import!
from IPython.utils import io
import pybullet as p
import pybullet_data
from time import sleep
import pyrosim.pyrosim as pyrosim
import numpy as np
import random
import constants as const
from collections import defaultdict

class Simulation:
    def __init__(self) -> None:
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.80665)
        self.world = World()
        self.robot = Robot()
    def run(self) -> None:
        for i in range(const.total_iterations):
            p.stepSimulation()
            self.robot.sense(i)
            self.robot.act(i)
            sleep(const.tickRateSeconds)
            print(i)
        if const.save:
            self.robot.save_sensors()
            self.robot.save_motors()
    def __del__(self):
        p.disconnect()


class World:
    def __init__(self, world_path = "world.sdf", plane_path = 'plane.urdf') -> None:
        if world_path:
            self.world = p.loadSDF(world_path)
        if plane_path:
            self.plane = p.loadURDF(plane_path)

class Robot:
    def __init__(self, body_path = 'body.urdf') -> None:
        self.motors = defaultdict(Motor)
        if body_path:
            self.id = p.loadURDF(body_path)
        pyrosim.Prepare_To_Simulate(self.id)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self) -> None:
        self.sensors = defaultdict(Sensor)
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = Sensor(linkName)
    
    def Prepare_To_Act(self) -> None:
        self.joints = defaultdict(Motor)
        for jointName in pyrosim.jointNamesToIndices:
            self.joints[jointName] = Motor(jointName)

    def sense(self, i: int) -> None:
        for sensor in self.sensors.values():
            sensor.values[i] = pyrosim.Get_Touch_Sensor_Value_For_Link(sensor.name)

    def act(self, i: int) -> None:
        for joint in self.joints.values():
            pyrosim.Set_Motor_For_Joint(bodyIndex = self.id,
                                    jointName = joint.name,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = joint.values[i],
                                    maxForce = 50)
    
    def save_sensors(self) -> None:
        for sensor in self.sensors.values():
            sensor.save()
    def save_motors(self) -> None:
        print('inside motors')
        for motor in self.joints.values():
            motor.save()

class Sensor:
    def __init__(self, name: str) -> None:
        self.name = name
        self.values = np.zeros(const.total_iterations)
    def save(self) -> None:
        const.npsave(const.savepath+self.name, self.values)
class Motor:
    def __init__(self, name: str) -> None:
        self.name = name
        self.Prepate_To_Act()
    def Prepate_To_Act(self) -> None:
        if self.name == b'Torso_backLeg':
            self.amplitude = const.backAmp
            self.frequency = const.backF
            self.phase = const.backPhase
        else:
            self.amplitude = const.frontAmp
            self.frequency = const.frontF
            self.phase = const.frontPhase
        self.values = np.sin(np.linspace(0, 2*np.pi, num=const.total_iterations)*self.frequency + self.phase)*self.amplitude
    def save(self) -> None:
        const.npsave(const.savepath+str(self.name)[2:-1], self.values)




        
