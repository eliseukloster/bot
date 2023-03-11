import pyrosim.pyrosim as pyrosim
import constants as const
from dataclasses import dataclass, field, InitVar
from random import choice, randint
import numpy as np
from functools import partial

def random_size() -> list[float]:
    #return [1,1,1]
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

def random_neuron(type: str) -> bool:
    return np.random.choice((True, False), p=(const.probability_neurons[type], 1-const.probability_neurons[type]))

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
    neuron: bool = field(default_factory = partial(random_neuron, type='motor'))

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
    neuron: bool = field(default_factory = partial(random_neuron, type='sensor'))

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
            size = self.size,
            color = const.colors[self.neuron]
        )
    def send_neuron(self, n: int) -> int:
        if self.neuron:
            pyrosim.Send_Sensor_Neuron(name = n , linkName = self.name_str)
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
            #position[axis] = 0

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
            #position[axis] = 0

    link = Link(
        name = name,
        position = position,
        upstream_joint_abs_position = upstream_joint_abs_position,
        joint_coord_axis = joint_coord_axis,
        joint_coord_axis_sign = joint_coord_axis_sign,
        size = size
    )

    return link


def Create_Robot(id: int, nLinks:int, links: list[Link] | None = None, joints: list[Joint] | None = None,
                 xtorso: float | None = None, ytorso: float | None = None, ztorso: float | None = None) -> tuple[list[Link], list[Joint]]:
    '''
    Creates a urdf robot with thre cube links and two joints.
    '''
    if links is None and joints is None:
        size = random_size()
        if xtorso is None:
            xtorso = 0
        if ytorso is None:
            ytorso = 0
        if ztorso is None:
            ztorso = size[2]/2
        link0 = Link(
            name = 0,
            position = [xtorso,ytorso,ztorso],
            upstream_joint_abs_position = None,
            coords_type = 'absolute',
            size = size
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
            #links[joint.parent].accept()
            links.append(link)

        zmin = 0
        for link in links:
            zmin = min(zmin, link.bounds['z'][0])
        pyrosim.Start_URDF(f'body{id}.urdf')
        for link in links:
            if link.coords_type == 'absolute':
                link.position[2] += -zmin
            link.send()
        for joint in joints:
            if joint.coords_type == 'absolute':
                joint.position[2] += -zmin
            joint.send()
        pyrosim.End()
        return links, joints
    else:
        pyrosim.Start_URDF(f'body{id}.urdf')
        for link in links:
            link.send()
        for joint in joints:
            joint.send()
        pyrosim.End()
        return links, joints

def Create_Brain(id: int, links: list[Link], joints: list[Joint], weights: np.ndarray | None = None) -> np.ndarray:
    '''
    Creates a neural network for the previously created robot.
    '''
    pyrosim.Start_NeuralNetwork(f'brain{id}.nndf')
    n = 0
    nsensors = 0
    nmotors = 0
    for link in links:
        if link.neuron:
            nsensors += 1
    for joint in joints:
        if joint.neuron:
            nmotors +=1

    if nsensors == 0:
        links[randint(0, len(links)-1)].neuron = True
        nsensors+=1
    if nmotors == 0:
        joints[randint(0, len(joints)-1)].neuron = True
        nmotors+=1

    for link in links:
        if link.neuron:
            n = link.send_neuron(n)
    for joint in joints:
        if joint.neuron:
            n = joint.send_neuron(n)

    if weights is None:
        weights = 2*np.random.randn(nsensors, nmotors)
    for sensor in range(nsensors):
        for motor in range(nsensors, nsensors+nmotors):
            pyrosim.Send_Synapse( sourceNeuronName = sensor , targetNeuronName = motor , weight = weights[sensor][motor-nsensors] )
    pyrosim.End()

    return weights

if __name__ == '__main__':
    pass