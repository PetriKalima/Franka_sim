Python interface for controlling the robot model.

----

Usage
-----

Import ``interface.py`` as a python module.

The ``Simpackage`` class takes a path to an xml model as an argument. 
It can then be used to control the robot model using four functions:

1. getState()
    Returns a numpy array with the positions and velocities of the joints.

#. setState(state)
    Takes a state dict with joint names as keys and their wanted values as values.

#. step(action)
    Takes in an action list, which includes a force for each joint that will be applied.

#. reset()
    Resets the robot to its initial state.
