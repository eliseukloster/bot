#import!
from IPython.utils import io
import pybullet as p
import pybullet_data
from time import sleep
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import numpy as np
import random
import supress
import constants as const
from collections import defaultdict

class Simulation:
    def __init__(self, connection='DIRECT') -> None:
        if connection == 'GUI':
            self.physicsClient = p.connect(p.GUI)
            self.useGUI = True
        else:
            self.physicsClient = p.connect(p.DIRECT)
            self.useGUI = False
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.80665)
        self.world = World()
        self.robot = Robot()

    def run(self) -> None:
        """
        Runs the simulation by stepping the physics engine
        and executing robot's actions.
        """
        for i in range(const.total_iterations):
            p.stepSimulation()
            self.robot.sense(i)
            self.robot.think()
            self.robot.act(i)
            if self.useGUI:
                sleep(const.tickRateSeconds)
                p.resetDebugVisualizerCamera( cameraDistance=9, cameraYaw=0, cameraPitch=-25, cameraTargetPosition=[-0.005*i,0,0])
        if const.save:
            self.robot.save_sensors()
            self.robot.save_motors()

    def get_fitness(self) -> None:
        """
        Evaluates the robot's fitness function.
        """
        self.robot.get_fitness()

    def __del__(self) -> None:
        """
        Disconnects from the physics client before deleting.
        """
        p.disconnect()

class World:
    """
    Loads sdf files to the world.
    """
    def __init__(self, world_path = "world.sdf", plane_path = 'plane.urdf') -> None:
        with supress.stdout_redirected():
            if world_path:
                self.world = p.loadSDF(world_path)
            if plane_path:
                self.plane = p.loadURDF(plane_path)

class Robot:
    """
    Creates a robot with a neural network.
    """
    def __init__(self, body_path = 'body.urdf') -> None:
        if body_path:
            self.id = p.loadURDF(body_path)
        pyrosim.Prepare_To_Simulate(self.id)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK('brain.nndf')

    def Prepare_To_Sense(self) -> None:
        """"
        Creates sensors for the robot.
        """
        self.sensors = defaultdict(Sensor)
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = Sensor(linkName)

    def Prepare_To_Act(self) -> None:
        """
        Creates joints for the robot.
        """
        self.motors = defaultdict(Motor)
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = Motor(jointName)

    def sense(self, i: int) -> None:
        """
        Captures the sensor values and stores them.
        """
        for sensor in self.sensors.values():
            sensor.values[i] = pyrosim.Get_Touch_Sensor_Value_For_Link(sensor.name)

    def think(self) -> None:
        """
        Steps the neural network.
        """
        self.nn.Update()
        #self.nn.Print()

    def act(self, i: int) -> None:
        """
        Actuates the motors based on neural network.
        """
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(neuronName)
                pyrosim.Set_Motor_For_Joint(bodyIndex = self.id,
                        jointName = bytes(jointName, 'utf-8'),
                        controlMode = p.POSITION_CONTROL,
                        targetPosition = desiredAngle,
                        maxForce = 50)
                self.motors[bytes(jointName, 'utf-8')].values[i] = desiredAngle

    def save_sensors(self) -> None:
        """
        Saves sensor values.
        """
        for sensor in self.sensors.values():
            sensor.save()
    def save_motors(self) -> None:
        """
        Saves motor positions.
        """
        for motor in self.motors.values():
            motor.save()

    def get_fitness(self) -> None:
        """
        Evaluates the fitness function of the robot.
        """
        xyz = p.getLinkState(self.id,0)[0]
        xCoordinateLink0 = xyz[0]
        with open('fitness.txt', 'w') as f:
            f.write(str(xCoordinateLink0))

class Sensor:
    """
    Creates a robot sensor.
    """
    def __init__(self, name: str) -> None:
        self.name = name
        self.values = np.zeros(const.total_iterations)
    def save(self) -> None:
        """
        Saves the sensor values.
        """
        const.npsave(const.savepath+self.name, self.values)

class Motor:
    """
    Creates a robot motor.
    """
    def __init__(self, name: str) -> None:
        self.name = name
        self.values = np.zeros(const.total_iterations)
    def save(self) -> None:
        """
        Saves the motor values.
        """
        const.npsave(const.savepath+str(self.name)[2:-1], self.values)



        
