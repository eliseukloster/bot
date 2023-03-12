from os import system
import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as const
from copy import deepcopy
from random import randint
from multiprocessing import Process
from multiprocessing.connection import Connection, Pipe
import simulate
import create
import pickle

class Climber:
    '''
    Evolves a robot by modifying the neurons weights
    and selecting the sets that best minimize a fitness function.
    '''
    def __init__(self, id: int, nlinks: int, evaluate_args: list = []) -> None:
        self.id = id
        self.evaluate_args = evaluate_args
        self.parent = Solution(self.id, nlinks, None, None, None)
    def evolve(self) -> float:
        '''
        Evolve generations and saves a history of all weights.
        '''
        self.parent.evaluate(self.evaluate_args)
        self.parent.join()
        
        # Save data for later use
        with open(const.savepath+f'robot{self.id}.pkl', 'wb') as f:
            pickle.dump((self.parent.links, self.parent.joints), f, pickle.HIGHEST_PROTOCOL)
        self.history = np.expand_dims(self.parent.weights, axis=2)
        for currGen in range(const.total_generations):
            if currGen % const.log_frequency == 0 and self.id % const.log_every == 0:
                print(f'PARENT {currGen}-{self.id}: ')
                print(self.parent.weights)
                print(f'fitness: {self.parent.fitness}')
            self.evolve_step()
            self.history = np.concatenate((self.history, np.expand_dims(self.parent.weights, axis=2)), axis=2)
        np.save(const.savepath+f'weights{self.id}', self.history, allow_pickle=False)
        return self.parent.fitness

    def evolve_step(self) -> None:
        '''
        Spawns and mutates a child. Selects the best between parent and child.
        '''
        self.spawn()
        self.mutate()
        self.child.evaluate(self.evaluate_args)
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
        if const.better_fitness(self.child.fitness, self.parent.fitness):
            self.parent = self.child

class Solution:
    def __init__(self,  id: int, nlinks:int, links: list | None, joints: list | None, weights: np.ndarray | None) -> None:
        self.id = id
        self.L, self.W, self.H = 1,1,1
        self.X, self.Y, self.Z = 0,0, self.H/2
        self.nlinks = nlinks
        self.links, self.joints, self.weights = links, joints, weights

    def evaluate(self, args: list = []) -> None:
        '''
        Calculates the fitness of the current solution.
        '''
        self.Create_World()
        self.Create_Robot()
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
        sensor = randint(0, self.weights.shape[0]-1)
        motor= randint(0, self.weights.shape[1]-1)

        self.weights[sensor, motor] = np.random.randn()

    def Create_World(self) -> None:
        '''
        Creates a new sdf world with a box.
        '''
        pyrosim.Start_SDF(f'world{self.id}.sdf')
        pyrosim.Send_Cube(name=f'Box', pos=[self.X-5,self.Y-5,self.Z], size=[self.L,self.W,self.H], color = '0.7 0.7 0.7 1.0')
        pyrosim.End()

    def Create_Robot(self,  xtorso: float | None = None, ytorso: float | None = None, ztorso: float | None = None) -> None:
        '''
        Creates a urdf robot with thre cube links and two joints.
        '''
        self.links, self.joints = create.Create_Robot(self.id, self.nlinks, self.links, self.joints, xtorso, ytorso, ztorso)

    def Create_Brain(self) -> None:
        '''
        Creates a neural network for the previously created robot.
        '''
        self.weights = create.Create_Brain(self.id, self.links, self.joints, self.weights)

if __name__ == '__main__':
    nlinks = const.nlinks()
    climber = Climber(0, nlinks)
    climber.evolve()
    weights = np.load(const.savepath+'weights0.npy', allow_pickle=False)
    with open(const.savepath+'robot0.pkl', 'rb') as f:
        links, joints = pickle.load(f)
    import supress
    with supress.stdout_redirected():
        s1 = Solution(0, nlinks, links, joints, weights[:,:,0])
        s1.evaluate(['--gui'])
        s1.join()
        s2 = Solution(0, nlinks, links, joints, weights[:,:,-1])
        s2.evaluate(['--gui'])
        s2.join()