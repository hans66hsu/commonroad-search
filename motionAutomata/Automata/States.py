class StartState:
    def __init__(self, xRange, yRange, velocity, vRange, oRange):
        """
        Initialisation of class StartState.

        :param xRange: the range of the position in x axis
        :param yRange: the range of the position in y axis
        :param velocity: the velocity if the state
        :param vRange: the range of the velocity
        :param oRange: the range of the orientation
        """
        self.xRange = xRange
        self.yRange = yRange
        self.velocity = velocity
        self.vRange = vRange
        self.oRange = oRange

    def __str__(self) -> str:
        """
        Returns the velocity information of StartState.
        """
        return "State: " + str(self.velocity) + " m/s"


class FinalState:
    def __init__(self, xPos, yPos, xRange, yRange, velocity, velocityRange, orientation, orientationRange):
        """
        Initialisation of class StartState.

        :param xPos: the position in x axis
        :param yPos: the position in y axis
        :param xRange: the range of the position in x axis
        :param yRange: the range of the position in y axis
        :param velocity: the velocity of the state
        :param velocityRange: the range of the velocity
        :param orientation: the orientation of the final state
        :param orientationRange: the range of the orientation
        """
        self.x = xPos
        self.y = yPos
        self.xRange = xRange
        self.yRange = yRange
        self.velocity = velocity
        self.vRange = velocityRange
        self.orientation = orientation
        self.oRange = orientationRange

    def __str__(self) -> str:
        """
        Returns the velocity, orientation and position information of the final state.
        """
        return "State: " + str(self.velocity) + " m/s - " + str(self.orientation) + " rad - " + str(self.x) + "~" + str(self.y) + " m"
