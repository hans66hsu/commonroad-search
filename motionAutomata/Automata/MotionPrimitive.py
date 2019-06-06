from Automata.States import StartState, FinalState
from commonroad.scenario.trajectory import Trajectory
from commonroad.scenario.trajectory import State as StateTupleFactory
from commonroad.scenario.trajectory import State
import numpy as np
from typing import List

class MotionPrimitive:
    def __init__(self, startState: StartState, finalState: FinalState, timeStepSize, trajectory: Trajectory):
        """
        Initialisation of a motion primitive.

        :param startState: start state of the primitive
        :param finalState: final state of the primitive
        """
        # self.id = id
        self.startState = startState
        self.finalState = finalState
        self.timeStepSize = timeStepSize
        self.trajectory = trajectory
        self.Successors = [] 
    def __str__(self):
        return "Primitive (" + str(self.startState) + ") => (" + str(self.finalState) + ")"

  
    def mirror(self) -> None:
        """
        Mirrors the current primitive so that it describes a primitive in the opposite direction.
        """
        self.finalState.y = - self.finalState.y
        self.finalState.orientation = - self.finalState.orientation
        for i in range(len(self.trajectory.state_list)):
            state = self.trajectory.state_list[i]
            self.trajectory.state_list[i].position[1] = - self.trajectory.state_list[i].position[1]
            state.orientation = - state.orientation
            self.trajectory.state_list[i] = state


    def checkConnectivityToNext(self, other: 'MotionPrimitive') -> bool: 
        """
        Any primitive whose first state's velocity is in the velocity range of the last state of the current primitive is considered as connected.
        Returns true if the two primitives are connected.

        :param other: the next motion primitive to be checked
        """
        if (self.finalState.velocity - self.finalState.vRange >= other.startState.velocity - other.startState.vRange
            and self.finalState.velocity + self.finalState.vRange <= other.startState.velocity + other.startState.vRange):
            return True
        return False
    '''
    # append the basic primitive to the state and return the new state but without intermediate steps
    def appendToState(self, state, state_tuple): ##TODO: to be finalized

        stateDict = state._asdict()

        pos = np.array([[0,0], [self.finalState.x, self.finalState.y]])
        ori = np.array([0,self.finalState.orientation])
        vel = np.array([state.velocity, self.finalState.velocity])

        traj = create_from_vertices(pos, 0, ori, vel)
        traj.translate_rotate(np.zeros(2), state.orientation)
        traj.translate_rotate(np.array(state.position), 0)

        newStateDict = traj.state_list[1]._asdict()
        newStateDict['time'] = stateDict['time'] * self.timeStepSize

        return state_tuple(**newStateDict)
    '''
    def appendTrajectoryToState(self, state: State) -> List[State]: 
        """
        Appends the primitive to the given state and returns the new list of states including the intermediate steps.

        :param state: the trajectory will be appended to this state
        """
        from copy import deepcopy
        traj = deepcopy(self.trajectory)
        traj.translate_rotate(np.zeros(2), state.orientation)
        traj.translate_rotate(state.position, 0)
        path = []
        traj.state_list.pop(0)

        num_timeSteps = int(10*self.timeStepSize)
 
        if num_timeSteps > 0:

            ratio = len(traj.state_list)/num_timeSteps
            index = len(traj.state_list)-1
            timeStamp = num_timeSteps
            while len(path) < num_timeSteps:
                newStateDict = None
                idx = round(index)
                newStateDict = deepcopy(traj.state_list[idx])
                newStateDict.time_step = int(state.time_step) + int(timeStamp)
                path.append(newStateDict)
                index -= ratio
                timeStamp -= 1

        path.reverse()

        return path

    def costs(self) -> float:
        """
        Returns euclidean distance between the last state of current primitive and [0,0].
        """
        pos1 = [0,0]
        pos2 = [self.finalState.x, self.finalState.y]
        return np.sqrt((pos1[0] - pos2[0]) * (pos1[0] - pos2[0]) + (pos1[1] - pos2[1]) * (pos1[1] - pos2[1]))
