from os import system
import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as const
from copy import deepcopy
from random import randint

class Climber():
    '''
    Evolves a robot by modifying the neurons weights
    and selecting the sets that best minimize a fitness function.
    '''
    def __init__(self) -> None:
        self.parent = Solution()
        self.history = self.parent.weights.reshape((1,-1))

    def evolve(self) -> None:
        '''
        Evolve generations and saves a history of all weights.
        '''
        self.parent.evaluate()
        for currGen in range(const.total_generations):
            print(f'PARENT {currGen}: ')
            print(self.parent.weights)
            self.evolve_step()
            self.history = np.vstack((self.history, self.parent.weights.reshape((1,-1))))
        np.savetxt('weights.csv', self.history, delimiter=',')

    def evolve_step(self) -> None:
        '''
        Spawn and mutates a child. Selects the best between parent and child.
        '''
        self.spawn()
        self.mutate()
        self.child.evaluate()
        print('')
        print(f'PARENT: {self.parent.fitness}; CHILD: {self.child.fitness}')
        print('='*15)
        self.select()

    def spawn(self) -> None:
        '''
        Spawns the child.
        '''
        self.child = deepcopy(self.parent)

    def mutate(self) -> None:
        '''
        Mutates the child.
        '''
        self.child.mutate()

    def select(self) -> None:
        '''
        Selects the best between parent and child.
        '''
        if self.child.fitness < self.parent.fitness:
            self.parent = self.child

class Solution():
    def __init__(self,  weights=None) -> None:
        if weights is None:
            self.weights = 2*np.random.randn(3, 2)
        else:
            self.weights = weights.reshape(3,2)
        self.L, self.W, self.H = 1,1,1
        self.X, self.Y, self.Z = 0,0, self.H/2

    def evaluate(self) -> None:
        '''
        Calculates the fitness of the current solution
        and saves it to a file fitness.txt.
        '''
        self.Create_World()
        self.Create_Robot(1.5,0,1.5)
        self.Create_Brain()
        system('python3 simulate.py')
        with open('fitness.txt', 'r') as f:
            self.fitness = float(f.read())     

    def mutate(self) -> None:
        '''
        Randomly select one of the weights to replace by a new random value. 
        '''
        self.weights[randint(0,2), randint(0,1)] = np.random.randn()

    def Create_World(self) -> None:
        '''
        Creates a new sdf world with a box.
        '''
        pyrosim.Start_SDF('world.sdf')
        pyrosim.Send_Cube(name=f'Box', pos=[self.X-5,self.Y-5,self.Z], size=[self.L,self.W,self.H])
        pyrosim.End()

    def Create_Robot(self, xtorso: float, ytorso: float, ztorso: float) -> None:
        '''
        Creates a urdf robot with thre cube links and two joints.
        '''
        pyrosim.Start_URDF('body.urdf')
        pyrosim.Send_Cube(name=f'Torso', pos=[xtorso, ytorso, ztorso], size=[self.L,self.W,self.H])
        pyrosim.Send_Joint( name = "Torso_FrontLeg",
                            parent= "Torso",
                            child = "FrontLeg",
                            type = "revolute",
                            position = [xtorso+self.L/2, ytorso, ztorso-self.H/2]
                            )
        pyrosim.Send_Cube(name=f'FrontLeg', pos=[self.L/2,0,-self.H/2], size=[self.L,self.W,self.H])
        pyrosim.Send_Joint( name = "Torso_BackLeg",
                            parent= "Torso",
                            child = "BackLeg",
                            type = "revolute",
                            position = [xtorso-self.L/2, ytorso, ztorso-self.H/2]
                            )
        pyrosim.Send_Cube(name=f'BackLeg', pos=[-self.L/2,0,-self.H/2], size=[self.L,self.W,self.H])
        pyrosim.End()

    def Create_Brain(self) -> None:
        '''
        Creates a neural network for the previously created robot.
        '''
        pyrosim.Start_NeuralNetwork("brain.nndf")
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
        pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")
        for sensor in [0,1,2]:
            for motor in [3,4]:
                pyrosim.Send_Synapse( sourceNeuronName = sensor , targetNeuronName = motor , weight = self.weights[sensor][motor-3] )
        pyrosim.End()

if '__name__' == '__main__':
    climber = Climber()
    climber.evolve()

    weights = np.loadtxt('weights.csv', delimiter=',')
    Solution(weights[0])
    system('python3 simulate.py -g')
    Solution(weights[-1])
    system('python3 simulate.py -g')
