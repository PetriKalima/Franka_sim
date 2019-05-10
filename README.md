# franka_sim

To open the door simulation use command:
    roslaunch franka_sim door.launch

To try the door opening demo use command:
    rosrun franka_sim door_demo


Issues:
    -door model
        -The door handle is so low that the robot ee cannot move through the door frame without collision while holding the handle.
    -robot_door.urdf.xacro
        -Create a "dummy" collision stl file for the handle_beam (handle_v5_beam_bin.stl) and remove comments from visual mesh. The dummy should be blank or atleast not represent the collisions accurately as a collision between a ee finger and the handle_beam will otherwise result in error.
