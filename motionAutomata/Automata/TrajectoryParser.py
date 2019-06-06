import numpy as np
import xml.etree.ElementTree as et
#from commonroad.scenario.trajectory import StateTupleFactory 
from commonroad.scenario.trajectory import State
from sphinx.util import xmlname_checker
from Automata.States import StartState, FinalState

class Trajectory:
    def __init__(self, id, startState: StartState, finalState: FinalState):
        self.id = id
        self.startState = startState
        self.finalState = finalState

    def checkConnectivityToNext(self, other: 'Trajectory') -> bool:
        """
        Checks if the connectivity condition to another trajectory is met, only the velocity constraint is checked. 
        Returns true if the two trajectories are connected.

        :param other: another trajectory
        """
        
        if (self.finalState.velocity - self.finalState.vRange >= other.startState.velocity - other.startState.vRange
            and self.finalState.velocity + self.finalState.vRange <= other.startState.velocity + other.startState.vRange):
            return True
        return False

# creates trajectory from a given xml node
class TrajectoryParser:
    @classmethod
    def createFromNode(self, xmlNode):
        """
        Returns a trajectory from the element found in xml. (e.g. the element of the list returned by xmlTree.findall('Trajectory'))
        """
        #   id = int(xmlNode.get('id'))
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
        return Trajectory(id, startState, finalState)


class TrajectorySet:
    def __init__(self):
        return

    def readFromXML(self, filename: str):
        """
        Reads all trajectories of the given file and stores them in the Trajectory array

        :param filename: the name of the file
        """
        xmlTree = et.parse(filename).getroot()
        self.numTrajectories = len(xmlTree.findall('Trajectory'))
        self.Trajectories = np.empty(self.numTrajectories, dtype=Trajectory)
        i = 0
        for t in xmlTree.findall('Trajectory'):
            self.Trajectories[i] = TrajectoryParser.createFromNode(t)
            self.Trajectories[i].id = i
            i = i + 1

    def createConnectivityLists(self):
        """
        Creates a connectivity list for every trajectory (let every trajectory has its corresponding successor list).
        """
        for t in self.Trajectories:
            t.Successors = []
            #print ('Trajectory[' + str(t.id) + ']')
            for i in range(self.numTrajectories):
                if t.checkConnectivityToNext(self.Trajectories[i]):
                    t.Successors.append(self.Trajectories[i])
             #       print ('Added successor ' + str(i))
        return

    def sortViaVelocity(self):
        #TODO: Sort and store the trajectories depending on their input velocity -> faster processing
        return