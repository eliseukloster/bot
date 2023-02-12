#import
import pyrosim.pyrosim as pyrosim
import numpy as np

L, W, H = 1,1,1
X, Y, Z = 0,0,H/2

def Create_World():
    pyrosim.Start_SDF('world.sdf')
    pyrosim.Send_Cube(name=f'Caixa', pos=[X-5,Y-5,Z], size=[L,W,H])
    pyrosim.End()
def Create_Robot2(xtorso, ytorso, ztorso):
    pyrosim.Start_URDF('body.urdf')
    pyrosim.Send_Cube(name=f'Torso', pos=[xtorso, ytorso, ztorso], size=[L,W,H])
    pyrosim.Send_Joint( name = "Torso_FrontLeg",
                        parent= "Torso",
                        child = "FrontLeg",
                        type = "revolute",
                        position = [xtorso+L/2, ytorso, ztorso-H/2]
                        )
    pyrosim.Send_Cube(name=f'FrontLeg', pos=[L/2,0,-H/2], size=[L,W,H])
    pyrosim.Send_Joint( name = "Torso_BackLeg",
                        parent= "Torso",
                        child = "BackLeg",
                        type = "revolute",
                        position = [xtorso-L/2, ytorso, ztorso-H/2]
                        )
    pyrosim.Send_Cube(name=f'BackLeg', pos=[-L/2,0,-H/2], size=[L,W,H])
    pyrosim.End()

def init_weights(sensor, motor):
    array = [1.184060212857539707e+00,-1.381134340846313702e+00,8.728293451667138436e-01,5.904321101350659129e-01,1.447882886858451679e+00,1.927693231665949192e+00]
    array = np.array(array).reshape(3,2)
    return array[sensor, motor]

def Create_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
    pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")
    pyrosim.Send_Motor_Neuron( name = 3 , jointName = "Torso_BackLeg")
    pyrosim.Send_Motor_Neuron( name = 4 , jointName = "Torso_FrontLeg")

    for sensor in [0,1,2]:
        for motor in [3,4]:
            pyrosim.Send_Synapse( sourceNeuronName = sensor , targetNeuronName = motor , weight = init_weights(sensor, motor-3) )
    pyrosim.End()

Create_World()
Create_Robot2(1.5, 0, 1.5)
Create_Brain()