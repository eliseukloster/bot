import os
import numpy as np

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

savepath = 'data/'
save = False
total_iterations = 50000
camera_period = 1
gravity = -9.80665 # CODATA :P
tickRateSeconds = 1/240
total_generations = 100
log_frequency = 10
log_every = 6
population = 12

nSensorNeurons = 9
nMotorNeurons = 8
motorRange = 0.2

meanLinkSize = 1
varianceLinkSize = 0.3
fLinkSize = abs

nLinks = 3

colors = {True: '0 0.5 1.0 1.0', False: '1.0 0 0 1.0'}