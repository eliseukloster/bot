import pyrosim.pyrosim as pyrosim
import constants as const
import numpy as np
def robot(xtorso: float, ytorso: float, ztorso: float, id: int) -> None:
    pyrosim.Start_URDF(f'body{id}.urdf')
    pyrosim.Send_Mesh(path='bot.dae')
    pyrosim.Send_Cube(name=f'Torso', pos=[xtorso, ytorso, ztorso], size=[self.L,self.W,self.H])
    pyrosim.Send_Joint( name = "Torso_FrontLeg",
                        parent= "Torso",
                        child = "FrontLeg",
                        type = "revolute",
                        position = [xtorso, ytorso+0.5, ztorso],
                        jointAxis='1 0 0'
                        )
    pyrosim.End()

def brain(weights: np.ndarray, id: int) -> None:
    pyrosim.Start_NeuralNetwork(f'brain{id}.nndf')
    pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
    pyrosim.Send_Sensor_Neuron(name = 8 , linkName = "LeftLowerLeg")
    pyrosim.Send_Motor_Neuron( name = 9 , jointName = "Torso_BackLeg")
    for sensor in range(const.nSensorNeurons):
        for motor in range(const.nSensorNeurons, const.nSensorNeurons+const.nMotorNeurons):
            pyrosim.Send_Synapse( sourceNeuronName = sensor , targetNeuronName = motor , weight = weights[sensor][motor-const.nSensorNeurons] )
    pyrosim.End()
