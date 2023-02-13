from classes import Simulation
import argparse

parser = argparse.ArgumentParser(
    prog='Simulate',
    description='Reads the local world, body, and brain files to simulate a robot',
    epilog='Eliseu Kloster'
)
parser.add_argument('-g', '--gui', default='', action='store_const', const=['GUI'])
args = parser.parse_args()
simulation = Simulation(*args.gui)
simulation.run()
simulation.get_fitness()
del simulation
