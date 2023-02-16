import search
import constants as const
from multiprocessing import Process
import numpy as np
import supress

def main():
    processes = []
    for i in range(const.population):
        climber = search.Climber(i)
        process = Process(target=climber.evolve)
        #with supress.stdout_redirected():
        process.start()
        processes.append(process)

    weightss = np.empty((const.population, 6))
    fitnesses = np.empty(const.population)
    for i, process in enumerate(processes):
        process.join()
        weights = np.loadtxt(f'weights{i}.csv', delimiter=',')
        weightss[i, :] = weights[-1]

        s = search.Solution(0, weights[-1])
        s.evaluate()
        s.join()
        fitnesses[i] = s.fitness
    return weightss, fitnesses

if __name__ == '__main__':
    weightss, fitnesses = main()
    print(weightss)
    print(fitnesses)
