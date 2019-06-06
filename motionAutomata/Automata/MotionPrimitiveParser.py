import numpy as np
import xml.etree.ElementTree as et
#from commonroad.scenario.trajectory import StateTupleFactory
from commonroad.scenario.trajectory import Trajectory, State
from sphinx.util import xmlname_checker
from Automata.MotionPrimitive import MotionPrimitive
from Automata.States import StartState, FinalState


# creates trajectory from a given xml node
class MotionPrimitiveParser:

    @classmethod
    def createFromNode(cls, xmlNode) -> MotionPrimitive:
        """
        Creates a motion primitive from the xml node.

        :param cls:
        :param xmlNode: cls and xmlNode are often given together as an element of the list returned by xmlTree.findall(...)

        """
        #   id = int(xmlNode.get('id'))
        timeStepSize = float(xmlNode.find('timeStepSize').text) #  * dt
        startXrange = float(xmlNode.find('Start').find('x').get('range'))
        startYrange = float(xmlNode.find('Start').find('y').get('range'))
        startVel = float(xmlNode.find('Start').find('velocity').text)
        startVelRange = float(xmlNode.find('Start').find('velocity').get('range'))
        startOrientationRange = float(xmlNode.find('Start').find('orientation').get('range'))
        startState = StartState(startXrange, startYrange, startVel, startVelRange, startOrientationRange)

        FinalNode = xmlNode.find('Final')
        finalXPos = float(FinalNode.find('x').text)
        finalXRange = float(FinalNode.find('x').get('range'))
        finalYPos = float(FinalNode.find('y').text)
        finalYRange = float(FinalNode.find('y').get('range'))
        finalVelocity = float(FinalNode.find('velocity').text)
        finalVelRange = float(FinalNode.find('velocity').get('range'))
        finalOrientation = float(FinalNode.find('orientation').text)
        finalOrientationRange = float(FinalNode.find('orientation').get('range'))

        finalState = FinalState(finalXPos, finalYPos, finalXRange, finalYRange, finalVelocity, finalVelRange,
                                finalOrientation, finalOrientationRange)

        pathNode = xmlNode.find('Path')
        trajectory = MotionPrimitiveParser.createTrajectoryFromPathStates(pathNode, startState, finalState, timeStepSize) # dt

        return MotionPrimitive(startState, finalState, timeStepSize, trajectory)

    @classmethod
    def createTrajectoryFromPathStates(cls, xmlNode, startState: StartState, finalState: FinalState, timeStepSize: float): # dt
        """
        Creates trajectory state list from the path values described in the xml file.

        :param cls:
        :param xmlNode: cls and xmlNode are often given together as the return value of xmlNode.find(...)
        :param startState: start State of the trajectory
        :param finalState: final State of the trajectory
        :param timeStepSize: time step size
        """
        vertices = []
        velocities = []
        orientations = []

        #insert start state
        vertices.append([0,0])
        velocities.append(startState.velocity)
        orientations.append(0)

        #insert trajectory states
        skipCounter = 0
        if xmlNode is not None:
            states = xmlNode.findall('State')
            for t in states:
                if skipCounter < (1 / timeStepSize) * (20 / np.max([startState.velocity, finalState.velocity])) :#(len(states) / (timeStepSize * 10)):# * 20 / ((startState.velocity + finalState.velocity) / 2):
                    skipCounter = skipCounter + 1
                    continue

                skipCounter = 0

                X = float(t.find('x').text)
                #Xrange = float(t.find('x').get('range'))
                Y = float(t.find('y').text)
                #Yrange = float(t.find('y').get('range'))
                Vel = float(t.find('velocity').text)
                #VelRange = float(t.find('velocity').get('range'))
                Orientation = float(t.find('orientation').text)
                #OrientationRange = float(t.find('orientation').get('range'))

                vertices.append([X,Y])
                velocities.append(Vel)
                orientations.append(Orientation)

        #insert final state
        vertices.append([finalState.x, finalState.y])
        velocities.append(finalState.velocity)
        orientations.append(finalState.orientation)

        trajectory = create_from_vertices(np.array(vertices), 0, np.array(orientations), np.array(velocities))

        return trajectory

def create_from_vertices(vertices: np.ndarray, t0: int, orientation: np.ndarray=None, velocity: np.ndarray=None) -> Trajectory:
    """
    Creates a trajectory from a set of given positions, orientations, velocities and a starting time step.

    :param vertices: a set of positions for every state of the trajectory
    :param t0: starting time step of the trajectory
    :param orientation: a set of orientations for every state of the trajectory
    :param velocity: a set of velocities for every state of the trajectory 
    """
    vertices = vertices.copy()
    if velocity is not None:
        velocity = velocity.copy()
    if orientation is None:
        orientation = 0
    else:
        orientation = orientation.copy()
    return _create_from_unpacked_states(vertices=vertices, t0=t0, orientation=orientation, velocity=velocity)

def _create_from_unpacked_states(vertices: np.ndarray, t0: int, orientation: np.ndarray, velocity: np.ndarray=None) -> Trajectory:
    """
    Supports creating a trajectory from a set of given positions, orientations, velocities and a starting time step.

    :param vertices: a set of orientations for every state of the trajectory
    :param t0: starting time step of the trajectory
    :param oriientation: a set of orientations for every state of the trajectory
    :param velocity: a set of velocities for every state of the trajectory
    """
    if len(orientation) != len(vertices):
        raise Exception('orientation array should have the same length as vertices array')
    if velocity is not None and len(vertices) != len(velocity):
        raise Exception('velocity array should have the same length as vertices array')

    #state_tuple = []
    state_list = list()
    for state_idx in range(len(vertices)):
        if velocity is None:
            state_list.append(State(position=vertices[state_idx], orientation=orientation[state_idx], time_step=t0 + state_idx))
        else:
            state_list.append(State(position=vertices[state_idx], orientation=orientation[state_idx], velocity=velocity[state_idx], time_step=t0 + state_idx))
    return Trajectory(t0, state_list)