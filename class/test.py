from IPython.utils import io
import pybullet as p
import pybullet_data
from time import sleep
import pyrosim.pyrosim as pyrosim
from pyrosim.neuralNetwork import NEURAL_NETWORK
import numpy as np
import random
import supress
import constants as const
from collections import defaultdict
from os import remove

pyrosim.Start_SDF(f'world.sdf')
pyrosim.Send_Cube(name=f'Box')
pyrosim.End()
#with supress.stdout_redirected():
client = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.80665)


p.loadSDF("world.sdf")
p.loadURDF("plane.urdf")
p.loadURDF("body0.urdf")

for i in range(const.total_iterations):
    p.stepSimulation()
    sleep(const.tickRateSeconds)

p.disconnect()
