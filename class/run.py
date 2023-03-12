import search
import numpy as np
import supress
import argparse
import constants as const
import pickle

parser = argparse.ArgumentParser()
parser.add_argument('-w', '--weights', type=int, default=[0], nargs=1, help='Selects the i-th robot to load. Default is 0.')
parser.add_argument('-r', '--random-neurons', action='store_true', help='Randomizes the weights of the robot')

if __name__ == '__main__':
    args = parser.parse_args()
    with supress.stdout_redirected():
        with open(const.savepath+f'robot{args.weights[0]}.pkl', 'rb') as f:
            links, joints = pickle.load(f)
        nlinks = len(links)
        if args.random_neurons:
            weights = None
        else:
            weightss = np.load(const.savepath+f'weights{args.weights[0]}.npy', allow_pickle=False)
            weights = weightss[:, :, -1]
        try:
            s = search.Solution(0, nlinks, links, joints, weights)
            s.evaluate(['--gui'])
            s.join()
            s.fitness
            print(f'weights{args.weights[0]}.npy — fitness: {s.fitness}')
        except EOFError as e:
            pass