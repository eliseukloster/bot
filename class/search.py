from os import system
import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as const
from copy import deepcopy
from random import randint
from multiprocessing import Process
from multiprocessing.connection import Connection, Pipe
import simulate

class Climber():
    '''
    Evolves a robot by modifying the neurons weights
    and selecting the sets that best minimize a fitness function.
    '''
    def __init__(self, id: int) -> None:
        self.id = id
        self.parent = Solution(self.id)
        self.history = self.parent.weights.reshape((1,-1))
    def evolve(self) -> float:
        '''
        Evolve generations and saves a history of all weights.
        '''
        self.parent.evaluate()
        self.parent.join()
        for currGen in range(const.total_generations):
            if currGen % const.log_frequency == 0 and self.id % const.log_every == 0:
                print(f'PARENT {currGen}-{self.id}: ')
                print(self.parent.weights)
                print(f'fitness: {self.parent.fitness}')
            self.evolve_step()
            self.history = np.vstack((self.history, self.parent.weights.reshape((1,-1))))
        np.savetxt(f'weights{self.id}.csv', self.history, delimiter=',')
        return self.parent.fitness

    def evolve_step(self) -> None:
        '''
        Spawns and mutates a child. Selects the best between parent and child.
        '''
        self.spawn()
        self.mutate()
        self.child.evaluate()
        self.child.join()
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
    def __init__(self,  id: int, weights: np.ndarray | None = None) -> None:
        self.id = id
        if weights is None:
            self.weights = 2*np.random.randn(const.nSensorNeurons, const.nMotorNeurons)
        else:
            self.weights = weights.reshape(const.nSensorNeurons,const.nMotorNeurons)
        self.L, self.W, self.H = 1,1,1
        self.X, self.Y, self.Z = 0,0, self.H/2

    def evaluate(self, args: list = []) -> None:
        '''
        Calculates the fitness of the current solution
        and saves it to a file fitness.txt.
        '''
        self.Create_World()
        self.Create_Robot(0,0,1)
        self.Create_Brain()
        self.parent_connection, child_connection = Pipe()
        self.sim = Process(target=simulate.main, args=(self.id, args, child_connection))
        self.sim.start()

    def join(self) -> None:
        self.sim.join()
        self.fitness = self.parent_connection.recv()
        del self.parent_connection
        del self.sim

    def mutate(self) -> None:
        '''
        Randomly select one of the weights to replace by a new random value. 
        '''
        self.weights[randint(0,const.nSensorNeurons-1), randint(0,const.nMotorNeurons-1)] = np.random.randn()

    def Create_World(self) -> None:
        '''
        Creates a new sdf world with a box.
        '''
        pyrosim.Start_SDF(f'world{self.id}.sdf')
        pyrosim.Send_Cube(name=f'Box', pos=[self.X-5,self.Y-5,self.Z], size=[self.L,self.W,self.H])
        pyrosim.End()

    def Create_Robot(self, xtorso: float, ytorso: float, ztorso: float) -> None:
        '''
        Creates a urdf robot with thre cube links and two joints.
        '''
        pass

    def Create_Brain(self) -> None:
        '''
        Creates a neural network for the previously created robot.
        '''
        pass 

if __name__ == '__main__':
    climber = Climber(0)
    climber.evolve()

    weights = np.loadtxt('weights0.csv', delimiter=',')
    import supress
    with supress.stdout_redirected():
        s1 = Solution(0, weights[0])
        s1.evaluate(['--gui'])
        s1.join()
        s2 = Solution(0, weights[-1])
        s2.evaluate(['--gui'])
        s2.join()
