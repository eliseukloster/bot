import search
import constants as const
from multiprocessing import Process
import numpy as np
import supress
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--climb', action='store_true', help='Run a new simulation instead of loading the current directory files.')

def main(climb: bool = True):
    if climb:
        processes = []
        for i in range(const.population):
            climber = search.Climber(i)
            process = Process(target=climber.evolve)
            #with supress.stdout_redirected():
            process.start()
            processes.append(process)

        for i, process in enumerate(processes):
            process.join()

    weightss = np.empty((const.population, 6))
    fitnesses = np.empty(const.population)
    for i in range(const.population):
        weights = np.loadtxt(f'weights{i}.csv', delimiter=',')
        weightss[i, :] = weights[-1]
        s = search.Solution(0, weights[-1])
        s.evaluate()
        s.join()
        fitnesses[i] = s.fitness
    return weightss, fitnesses

if __name__ == '__main__':
    import time
    args = parser.parse_args()
    start = time.perf_counter()
    weightss, fitnesses = main(args.climb)
    elapsed = time.perf_counter() - start
    print(f'TOTAL TIME: {elapsed:0.2f} seconds.')
    print('WEIGHT')
    print(weightss)
    print('FITNESS')
    print(fitnesses)
    with supress.stdout_redirected():
        s = search.Solution(0, weightss[np.argmin(fitnesses)])
        s.evaluate(['--gui'])
        s.join()
