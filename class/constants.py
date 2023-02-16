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
total_iterations = 500
gravity = -9.80665 # CODATA :P
tickRateSeconds = 1/240
total_generations = 2
population = 2
