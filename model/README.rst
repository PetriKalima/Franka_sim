Use door.py to modify and test the door model.

**NOTE:** Use python version 3!

----

**Usage:**

See help message and parameters:

    ``python3 door.py -h``

    ``python3 door.py run -h``

    ``python3 door.py modify -h``

----

Modifying the door:

    ``python3 door.py modify``

The *modify* subcommand can be used to modify the door's parameters and has the following arguments:
  -h, --help            show this help message and exit
  --default             Reset the door to default settings.
  --mass MASS           Set the mass of the door.
  --stiffness STIFFNESS
                        Set the stiffness of the handle.
  --scale SCALE         Set the scale of the door.
  --xml XML             Set the XML file to modify.
  --xacro XACRO         Set the xacro file to modify.
  --xml_dest XML_DEST   File to save the modified XML in.
  --xacro_dest XACRO_DEST
                        File to save the modified xacro in.

Multiple of these arguments can be given at once. The destinations will by default be the same files as the sources.

----

Running the simulation:

    ``python3 door.py run``

The *run* subcommand is used to run mujoco and test the model and has the following arguments:
  -h, --help     show this help message and exit
  --time TIME    Number of frames to simulate.
  --xml XML      Load the model from another xml file.
  --info  Print simulation info to console.

