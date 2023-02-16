from classes import Simulation
from multiprocessing.connection import Connection
import argparse
parser = argparse.ArgumentParser(
    prog='Simulate',
    description='Reads the local world, body, and brain files to simulate a robot',
    epilog='Eliseu Kloster'
)
parser.add_argument('-g', '--gui', default='', action='store_const', const=['GUI'])

def main(args=None, connection: Connection | None = None):
    if args:
        args = parser.parse_args(args)
    else:
        args = parser.parse_args()

    simulation = Simulation(*args.gui)
    simulation.run()
    fitness = simulation.get_fitness()
    if connection:
        connection.send(fitness)
        connection.close()
    del simulation

if __name__ == '__main__':
    main()