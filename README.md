# franka_sim

The franka_sim is a simulation environment built for the Franka Panda robot. The environment includes a door, which is based on an actual door found in the Intelligent Robotics research lab at Aalto University, and the Panda robot. The environment is meant to be used by the research group for reinforcement learning.

Features:
* A dynamical model of the door in 1:1 scale.
* Fully parametrisable door for testing other doors.
* An OpenAI-style python interface for reinforcement learning.
* A demo for validation of the environment.

Instructions:
* To open the door simulation use command:
    * roslaunch franka_sim door.launch

* To try the door opening demo use command:
    * rosrun franka_sim door_demo

    * To modify the demo there are some variables in the door_demo.cpp that can be easily changed. Alterations can be made to the grasping location on the handle, the rotation goals for turning the angle and opening the door, and the number of intermediate points. Further explanation is found in the door_demo.cpp.

* How to use the door parametrisation?

* How to use the interface?

Issues:
* The default door model size (same size as real) is such that the robot cannot open the door beyond 20 degrees, as the end effector does not fit through the door.

* In the robot_door.urdf.xacro the handle_beam collision box is a "dummy", and does not represent the actual handle collision box. This was done to circumvent the collision errors in moveit between the robot fingers and the handle.

