import search
import constants as const
from multiprocessing import Process
import numpy as np
import supress
import argparse
import pickle

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--climb', action='store_true', help='Run a new simulation instead of loading the current directory files.')

def main(climb: bool = True) -> tuple[np.ndarray, float]:
    '''
    Spawns multiple hill climbers to parallelize computation.
    '''
    nlinkss = []
    if climb:
        processes = []
        for i in range(const.population):
            nlinks = const.nlinks()
            nlinkss.append(nlinks)
            climber = search.Climber(i, nlinks)
            process = Process(target=climber.evolve)
            #with supress.stdout_redirected():
            process.start()
            processes.append(process)

        for i, process in enumerate(processes):
            process.join()

    weightsss = []
    linkss = []
    jointss = []
    fitnesses = []
    nlinkss = []
    for i in range(const.population):
        
        weightss = np.load(const.savepath+f'weights{i}.npy', allow_pickle=False)
        weights = weightss[:, :, -1]
        with open(const.savepath+f'robot{i}.pkl', 'rb') as f:
            links, joints = pickle.load(f)
        nlinks = len(links)
        s = search.Solution(0, nlinks, links, joints, weights)
        s.evaluate()
        s.join()
        fitnesses.append(s.fitness)
        weightsss.append(weights)
        linkss.append(links)
        jointss.append(joints)
        nlinkss.append(nlinks)

    return nlinkss, linkss, jointss, weightsss, fitnesses

if __name__ == '__main__':
    import time
    args = parser.parse_args()
    start = time.perf_counter()
    nlinkss, linkss, jointss, weightss, fitnesses = main(args.climb)
    elapsed = time.perf_counter() - start
    print(f'TOTAL TIME: {elapsed:0.2f} seconds.')
    print('WEIGHT')
    print(np.asarray(weightss))
    print('FITNESS')
    print(np.asarray(fitnesses))
    with supress.stdout_redirected():
        try:
            random_id = np.random.choice(range(const.population))
            s = search.Solution(0, nlinkss[random_id], linkss[random_id], jointss[random_id], weightss[random_id])
            s.evaluate(['--gui'])
            s.join()
        except EOFError as e:
            pass
        try:
            random_id = np.random.choice(range(const.population))
            s = search.Solution(0, nlinkss[random_id], linkss[random_id], jointss[random_id], weightss[random_id])
            s.evaluate(['--gui'])
            s.join()
        except EOFError as e:
            pass
        try:
            best_id = np.argmin(fitnesses)
            s = search.Solution(0, nlinkss[best_id], linkss[best_id], jointss[best_id], weightss[best_id])
            s.evaluate(['--gui'])
            s.join()
        except EOFError as e:
            pass
