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
    Takes a state dict with joint names
    (use the same names as the example)
    as keys and their wanted values as values.
    Remember that the values have to be within the joint limits.
    
    Example::

        sim.setState({
            "lumi_joint1": 0,
            "lumi_joint2": 0,
            "lumi_joint3": 0,
            "lumi_joint4": -0.1,
            "lumi_joint5": 0,
            "lumi_joint6": np.pi/2,
            "lumi_joint7": 0,
            "lumi_finger_joint1": 0.01,
            "lumi_finger_joint2": 0.01,
            "door_hinge": 0,
            "handle_core": 0 
            })

#. step(action)
    Takes in an array with a force for each joint that will be applied.
    The order of the forces has to be the same as the order of the joints.
    
    Example::

        sim.step([
            1,
            -2,
            3,
            1,
            0.12,
            0,
            2.321,
            0,
            0,
            0,
            0
        ])

#. reset()
    Resets the robot to its initial state.
