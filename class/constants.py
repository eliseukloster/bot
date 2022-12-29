import os
import numpy as np
def npsave(name, data):
    i=1
    while(os.path.exists(f'{name}{i}.npy')): i+=1
    np.save(f'{name}{i}', data)
    print(f'Numpy ndarray saved: {name}{i}.npy')

savepath = 'data/'
save = False

total_iterations = 1000
gravity = -9.80665 # CODATA :P

backAmp, backF, backPhase = np.pi/4, 20, 0
frontAmp, frontF, frontPhase = np.pi/4, 10, np.pi/2

tickRateSeconds = 1/120