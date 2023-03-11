import pyrosim.pyrosim as pyrosim
import constants as const
from dataclasses import dataclass, field, InitVar
from random import choice
import numpy as np

def random_size() -> list[float]:
    return [1,1,1]
    return [const.fLinkSize(const.varianceLinkSize*np.random.randn()+const.meanLinkSize) for i in [0,1,2]]

def random_axis() -> list[float]:
    return choice(([1,0,0], [0,1,0], [0,0,1]))

def collide_1d(boundsa, boundsb, direction):
    if boundsa[direction][0] == boundsb[direction][0]:
        return True
    if ((boundsa[direction][0] < boundsb[direction][0] and boundsb[direction][0] < boundsa[direction][1]) or 
    (boundsb[direction][0] < boundsa[direction][0] and boundsa[direction][0] < boundsb[direction][1])):
        return True
    else:
        return False



@dataclass
class Joint:
    parent: int
    child: int
    position: list[float]
    face_direction: int
    face_sign: int
    type: str = 'revolute'
    prev_abs_position: InitVar[list[int] | None] = None
    axis: list[int] = field(default_factory=random_axis)
    coords_type: str = 'relative'
    neuron: bool = field(default_factory = lambda: choice([True, False]))

    def __post_init__(self, prev_abs_position: list[int] = None):
        self.name_str = f'{self.parent}_{self.child}'
        self.axis_str = ' '.join(map(str, self.axis))
        self.parent_str, self.child_str = str(self.parent), str(self.child)
        if self.coords_type == 'relative':
            self.position_abs = [self.position[i]+prev_abs_position[i] for i in (0,1,2)]
        else:
            self.position_abs = self.position

    def send(self) -> None:
        pyrosim.Send_Joint(
            name = self.name_str,
            parent = self.parent_str,
            child = self.child_str,
            type = self.type,
            position = self.position,
            jointAxis = self.axis_str
        )
    def send_neuron(self, n: int) -> int:
        if self.neuron:
            pyrosim.Send_Motor_Neuron(name = n , jointName = self.name_str)
            return n+1
        else:
            return n

    def choose_link_position_size(self) -> tuple[list[int],list[int]]:
        size = random_size()

        position = [0,0,0]
        #face_direction = choice((0,1,2))

        #position[face_direction] = choice((-1,1)) * size[face_direction]/2
        position[self.face_direction] = self.face_sign * size[self.face_direction]/2

        for direction in (0,1,2):
            if direction != self.face_direction:
                position[direction] = choice((-1,1))  * np.random.rand() * size[direction]/2

        return position, size, self.face_sign, self.face_direction


@dataclass
class Link:
    name: int
    position: list[float]
    face_direction: int
    face_sign: int
    prev_abs_position: InitVar[list[int] | None] = None
    size: list[float] = field(default_factory = random_size)
    neuron: bool = field(default_factory = lambda: choice((True, False)))
    coords_type: str = 'relative'

    def __post_init__(self, prev_abs_position: list[int] = None):
        self.name_str = str(self.name)
        if self.coords_type == 'relative':
            self.position_abs = [self.position[i]+prev_abs_position[i] for i in (0,1,2)]
        else:
            self.position_abs = self.position

        boundx = (self.position_abs[0] - self.size[0]/2, self.position_abs[0] + self.size[0]/2)
        boundy = (self.position_abs[1] - self.size[1]/2, self.position_abs[1] + self.size[1]/2)
        boundz = (self.position_abs[2] - self.size[2]/2, self.position_abs[2] + self.size[2]/2)

        self.bounds = {'x': boundx, 'y': boundy, 'z': boundz}

    def send(self) -> None:
        pyrosim.Send_Cube(
            name = self.name_str,
            pos = self.position,
            size = self.size
        )
    def send_neuron(self, n: int) -> int:
        if self.neuron:
            pyrosim.Send_Sensor_Neuron(name = n , jointName = self.name_str)
            return n+1
        else:
            return n
        
    def choose_joint_position(self) -> tuple[list[int],int,int]:
        position = [0,0,0]

        found = False
        while not found:
            face_direction = choice((0,1,2))
            face_sign = choice((-1,1))
            if face_direction != self.face_direction and face_sign != -self.face_sign:
                found = True

        position[face_direction] = face_sign * self.size[face_direction]/2

        for direction in (0,1,2):
            if direction != face_direction:
                position[direction] = choice((-1,1))  * np.random.rand() * self.size[direction]/2
        return position, face_sign, face_direction
    
    def check_collision(self, other) -> bool:
        boundsa = self.bounds
        boundsb = other.bounds

        if (collide_1d(boundsa, boundsb, 'x') and 
            collide_1d(boundsa, boundsb, 'y') and 
            collide_1d(boundsa, boundsb, 'z')):
            print(boundsa)
            print(boundsb)
            return True
        else:
            print(boundsa)
            print(boundsb)
            return False 

def check_all_collisions(self: Link, links: list[Link]):
    collide = False
    for other in links:
        collide = self.check_collision(other) or collide
        print(collide)
        if collide:
            return collide
    return collide

nLinks = 4


def Create_Robot(self, xtorso: float, ytorso: float, ztorso: float) -> None:
    '''
    Creates a urdf robot with thre cube links and two joints.
    '''

    link0 = Link(name = 0, position = [0,0,0.5], coords_type ='absolute', face_direction=2, face_sign=1)
    joint0 = Joint(parent = 0, child = 1, position = [0,0,1], coords_type = 'absolute', face_direction=2, face_sign=1)
    links = [link0]
    joints = [joint0]

    for i in range(1,nLinks-1):
        collide  = True
        count = 0
        while collide and count < 20:
            count +=1
            position, size, face_sign, face_direction = joints[i-1].choose_link_position_size()
            link = Link(name=i, position=position, size=size, prev_abs_position = joints[i-1].position_abs, face_sign=face_sign, face_direction=face_direction)
            print(link)
            collide = check_all_collisions(link, links)
        links.append(link)

        position, face_sign, face_direction = link.choose_joint_position()
        joint = Joint(parent = i, child = i+1, position = position, prev_abs_position = joints[i-1].position_abs, face_sign=face_sign, face_direction=face_direction)
        joints.append(joint)
        print(joint)

    collide  = True
    count = 0
    while collide and count < 20:
        count +=1
        positionn, sizen, face_signn, face_directionn = joints[nLinks-2].choose_link_position_size()
        link = Link(name=nLinks-1, position=positionn, size=sizen, prev_abs_position = joints[nLinks-2].position_abs, face_sign=face_signn, face_direction=face_directionn, )
        collide = check_all_collisions(link, links)
    
    links.append(link)
    print(links)
    print(joints)

    pyrosim.Start_URDF(f'body{self.id}.urdf')
    for link in links:
        link.send()
    for joint in joints:
        joint.send()
    pyrosim.End()

def Create_Brain(self) -> None:
    '''
    Creates a neural network for the previously created robot.
    '''
    pyrosim.Start_NeuralNetwork(f'brain{self.id}.nndf')
    for sensor in range(const.nSensorNeurons):
        for motor in range(const.nSensorNeurons, const.nSensorNeurons+const.nMotorNeurons):
            pyrosim.Send_Synapse( sourceNeuronName = sensor , targetNeuronName = motor , weight = self.weights[sensor][motor-const.nSensorNeurons] )
    pyrosim.End()

if __name__ == '__main__':
    pass