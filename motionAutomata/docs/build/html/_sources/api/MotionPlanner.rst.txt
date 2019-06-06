Module Motion Planner
======================
The motion planner module in commonroad_search is based on search. The motion planner class is 
used to construct your planner, where you can define your own cost function and search algorithm in "calc_heuristic_cost" and "search_alg". You can use all available measurements offered in this module to construct your own cost function. The priority queue class is a helper class for defining the search algorithm.

The starting node (start state of the planning problem) of the search tree is defined by the loaded scenario (a scenario can have more than one planning problems with different initial state and goal state). Other nodes of the search tree consist of different motion primitives. Each motion primitive is a short trajectory which consists of several states. The aim is to search the tree and find a path (a combination of primitives) that satisfies the conditions of the goal state without causing any collisions.


.. automodule:: Automata.MotionPlanner


Motion Planner
---------------

``MotionPlanner`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: MotionPlanner
   :members:
   :member-order: bysource

Priority Queue
---------------

``PriorityQueue`` class
^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: PriorityQueue
   :members:
   :member-order: bysource

Other Functions
---------------

.. autofunction:: findClosestVertex
.. autofunction:: calcAngleOfPosition
.. autofunction:: distToClosestPointOnLine
.. autofunction:: findDistanceToNearestPoint
.. autofunction:: calc_travelled_distance
.. autofunction:: euclideanDistance
.. autofunction:: manhattanDistance
.. autofunction:: chebyshevDistance
.. autofunction:: sumOfSquaredDifference
.. autofunction:: meanAbsoluteError
.. autofunction:: meanSquaredError
.. autofunction:: canberraDistance
.. autofunction:: cosineDistance
.. autofunction:: distance
.. autofunction:: curvature_of_polyline
.. autofunction:: orientation_diff
.. autofunction:: length_of_polyline



