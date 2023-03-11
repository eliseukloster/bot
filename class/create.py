import pyrosim.pyrosim as pyrosim
import constants as const
from dataclasses import dataclass, field, InitVar
from random import choice, randint
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
    joint_coord_axis: int
    joint_coord_axis_sign: int
    upstream_joint_abs_position: InitVar[list[int] | None] = None
    type: str = 'revolute'
    axis: list[int] = field(default_factory=random_axis)
    coords_type: str = 'relative'
    neuron: bool = field(default_factory = lambda: choice([True, False]))

    def __post_init__(self, upstream_joint_abs_position: list[int] | None):
        self.name_str = f'{self.parent}_{self.child}'
        self.axis_str = ' '.join(map(str, self.axis))
        self.parent_str, self.child_str = str(self.parent), str(self.child)
        if self.coords_type == 'relative':
            self.position_abs = [self.position[i]+upstream_joint_abs_position[i] for i in (0,1,2)]
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


@dataclass
class Link:
    name: int
    position: list[float]
    joint_coord_axis: int = None
    joint_coord_axis_sign: int = None
    upstream_joint_abs_position: list[int] | None = None
    size: list[float] = field(default_factory = random_size)
    coords_type: str = 'relative'
    neuron: bool = field(default_factory = lambda: choice((True, False)))

    def __post_init__(self):
        self.name_str = str(self.name)
        if self.coords_type == 'relative':
            self.position_abs = [self.position[i]+self.upstream_joint_abs_position[i] for i in (0,1,2)]
        else:
            self.position_abs = self.position

        boundx = (self.position_abs[0] - self.size[0]/2, self.position_abs[0] + self.size[0]/2)
        boundy = (self.position_abs[1] - self.size[1]/2, self.position_abs[1] + self.size[1]/2)
        boundz = (self.position_abs[2] - self.size[2]/2, self.position_abs[2] + self.size[2]/2)

        self.bounds = {'x': boundx, 'y': boundy, 'z': boundz}

        self.free_directions = set([(i, 1) for i in (0,1,2)] + [(i,-1) for i in (0,1,2)])
        if self.joint_coord_axis is not None and self.joint_coord_axis_sign is not None:
            self.free_directions.remove((self.joint_coord_axis, self.joint_coord_axis_sign))

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
        
    def free_direction(self) -> tuple[int, int]:
        free = choice(list(self.free_directions))
        self.free = free
        return free
    def accept(self) -> None:
        self.free_directions.remove(self.free)

    def check_collision(self, other) -> bool:
        boundsa = self.bounds
        boundsb = other.bounds

        if (collide_1d(boundsa, boundsb, 'x') and 
            collide_1d(boundsa, boundsb, 'y') and 
            collide_1d(boundsa, boundsb, 'z')):
            return True
        else:
            return False 

def check_all_collisions(self: Link, links: list[Link]):
    collide = False
    for other in links:
        collide = self.check_collision(other) or collide
        if collide:
            return collide
    return collide


def joint_from_links(links: list[Link]):

    total_links = len(links)
    
    child = total_links
    parent = randint(0,total_links-1)
    parent_link = links[parent]
    
    position = [0,0,0]

    # Choose a free direction for joitns in the parent link
    joint_coord_axis, joint_coord_axis_sign = parent_link.free_direction()

    # Add coordinates relative to the center of the parent link
    position[joint_coord_axis] = joint_coord_axis_sign * parent_link.size[joint_coord_axis]/2

    for axis in (0,1,2):
        if axis != joint_coord_axis:
            position[axis] = choice((-1,1)) * np.random.rand() * parent_link.size[axis]/2

    # 
    if parent == 0:
        coords_type = 'absolute'
        position = [position[i] + parent_link.position_abs[i] for i in (0,1,2)]
        upstream_joint_abs_position = None
    else:
        coords_type = 'relative'
        position = [position[i] + parent_link.position[i] for i in (0,1,2)]
        upstream_joint_abs_position = parent_link.upstream_joint_abs_position
    
    joint = Joint(
        parent = parent,
        child = child,
        position = position,
        coords_type = coords_type,
        joint_coord_axis = joint_coord_axis,
        joint_coord_axis_sign = joint_coord_axis_sign,
        upstream_joint_abs_position = upstream_joint_abs_position
    )

    return joint
    

def link_from_joint(joint: Joint):
    name = joint.child
    upstream_joint_abs_position = joint.position_abs
    joint_coord_axis = joint.joint_coord_axis
    joint_coord_axis_sign = joint.joint_coord_axis_sign
    size = random_size()

    position = [0,0,0]
    # Add coordinates relative to the upstream joint
    position[joint_coord_axis] = joint_coord_axis_sign * size[joint_coord_axis]/2

    for axis in (0,1,2):
        if axis != joint_coord_axis:
            position[axis] = choice((-1,1)) * np.random.rand() * size[axis]/2

    link = Link(
        name = name,
        position = position,
        upstream_joint_abs_position = upstream_joint_abs_position,
        joint_coord_axis = joint_coord_axis,
        joint_coord_axis_sign = joint_coord_axis_sign,
        size = size
    )

    return link


nLinks =   20
def Create_Robot(self, xtorso: float, ytorso: float, ztorso: float) -> None:
    '''
    Creates a urdf robot with thre cube links and two joints.
    '''
    link0 = Link(
        name = 0,
        position = [0,0,0.5],
        upstream_joint_abs_position = None,
        coords_type = 'absolute',
    )
    links = [link0]
    joints = []
    for i in range(1, nLinks):
        joint = joint_from_links(links)
        link = link_from_joint(joint)
        collide = check_all_collisions(link, links)
        while collide:
            joint = joint_from_links(links)
            link = link_from_joint(joint)
            collide = check_all_collisions(link, links)
        joints.append(joint)
        links[joint.parent].accept()
        links.append(link)

    zmin = 0
    for link in links:
        zmin = min(link.bounds['z'][0], zmin)
    print(-zmin)
    links[0].position[2] += -zmin
    joints[0].position[2] += -zmin
    pyrosim.Start_URDF(f'body{self.id}.urdf')
    for link in links:
        link.send()
        print(link.position)
        print(link.bounds)
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