import os
import numpy as np
import pybullet as p

def npsave(name: str, data: np.ndarray) -> None:
    '''
    Saves a NumPy ndarray to a .npy file and 
    appends a number to the original name to avoid
    conflicts.
    '''
    i=1
    while(os.path.exists(f'{name}{i}.npy')): i+=1
    np.save(f'{name}{i}', data)
    print(f'NumPy ndarray saved: {name}{i}.npy')

def fitness(self) -> float:
    '''
    Evaluates fitness of a robot object self.
    ''' 
    xyz = p.getBasePositionAndOrientation(self.id)[0]
    xCoordinate = xyz[0]
    self.xyz = xyz
    return xCoordinate

def better_fitness(self: float, other: float) -> bool:
    ''''
    Compares fitness values.
    '''
    return self < other

def nlinks() -> int:
    '''
    Defines the probability distribution for number of links in a robot.
    '''
    ns = [2, 3, 4, 5, 6, 7, 8, 9, 10]
    distribution = [2, 3, 4, 3.5, 2, 2, 1, 1, 1]
    distribution = list(map(lambda x: x/sum(distribution), distribution))
    choice = np.random.choice(ns, p=distribution)
    return choice

savepath = 'data/'
save = False
total_iterations = 10000
camera_period = 1
gravity = -9.80665 # CODATA :P
tickRateSeconds = 1/240
total_generations = 500
log_frequency = 50
log_every = 25
population = 100

motorRange = 0.2

npedes = 4
total_generations = 500
log_frequency = 50
log_every = 25
population = 100

motorRange = 0.2

meanLinkSize = 1
varianceLinkSize = 0.4
fLinkSize = abs

probability_neurons = {'motor': 0.9, 'sensor': 0.7}
colors = {True: '0 0.5 1.0 1.0', False: '1.0 0 0 1.0'}

cameraDistance=7
cameraYaw=0
cameraPitch=-60
