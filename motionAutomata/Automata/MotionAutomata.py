import numpy as np
import xml.etree.ElementTree as et
from sphinx.util import xmlname_checker

from Automata.MotionPrimitiveParser import MotionPrimitiveParser
from Automata.MotionPrimitive import MotionPrimitive

class MotionAutomata:
    def __init__(self, state_tuple, dt):
        self.state_tuple = state_tuple
        self.numPrimitives = 0
        self.Primitives = []#np.empty(0, dtype=MotionPrimitive)
        #self.dt = dt
        return

    def readFromXML(self, filename: str) -> None:
        """
        Reads all MotionPrimitives from the given file and stores them in the primitives array.

        :param filename: the name of the file to be read from
        """
        xmlTree = et.parse(filename).getroot()
        self.numPrimitives = self.numPrimitives + len(xmlTree.findall('Trajectory'))
        #self.Primitives = np.empty(self.numPrimitives, dtype=MotionPrimitive)
        #i = 0
        for t in xmlTree.findall('Trajectory'):
            self.Primitives.append(MotionPrimitiveParser.createFromNode(t)) # self.dt
            #self.Primitives[i].id = i
            #i = i + 1
        return

    def createConnectivityListPrimitive(self, primitive: MotionPrimitive)->None:
        """
        Creates the successor list for a single primitive and stores them in a successor list of the given primitive.

        :param primitive: the primitive to be connected
        """
        for i in range(self.numPrimitives):
            if primitive.checkConnectivityToNext(self.Primitives[i]):
                primitive.Successors.append(self.Primitives[i])

    def createConnectivityLists(self)->None:
        """
        Creates a connectivity list for every primitive (let every primitive has its corresponding successor list).
        """
        for t in self.Primitives:
            self.createConnectivityListPrimitive(t)
        return

    def createMirroredPrimitives(self)->None:
        """
        Creates the mirrored motion primitives since the file to load primitives by default only contains left curves. This function computes the right curves.
        """
        import copy
        oldPrimitives = self.Primitives
        self.numPrimitives = 2 * self.numPrimitives
        self.Primitives = np.empty(self.numPrimitives, dtype=MotionPrimitive)
        for i in range(len(oldPrimitives)):
            self.Primitives[i] = oldPrimitives[i]
            newPrim = copy.deepcopy(self.Primitives[i])
            newPrim.mirror() 
            self.Primitives[i + len(oldPrimitives)] = newPrim
        return
