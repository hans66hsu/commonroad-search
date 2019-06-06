Module Motion Primitive
========================

A primitive is a short trajectory. It consists of a consecutive list of states (position, velocity, orientation, time step). All motion primitives start at position [0,0] and time step = 0. A typical usage is to translate it to the current state (function: translate_primitive_to_current_pos). Afterwards you can append the translated primitive to your current path: append_path(currentPath, translated_primitve).

Motion Primitive
-----------------

.. automodule:: Automata.MotionPrimitive

``MotionPrimitive`` class
^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. autoclass:: MotionPrimitive
   :members:
   :member-order: bysource


