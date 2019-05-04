import mujoco_py


class Simpackage:

    def __init__(self, modelPath):
        self.model = mujoco_py.load_model_from_path(modelPath)
        self.sim = mujoco_py.MjSim(self.model)
        self.initState = self.sim.get_state()
        self.joints = [ "lumi_joint1",
                        "lumi_joint2",
                        "lumi_joint3",
                        "lumi_joint4",
                        "lumi_joint5",
                        "lumi_joint6",
                        "lumi_joint7",
                        "lumi_finger_joint1",
                        "lumi_finger_joint2",
                        "door_hinge",
                        "handle_core"
                        ]
        

    def view(self):
        self.viewer = mujoco_py.MjViewer(self.sim)
        while True:
            self.viewer.render()

    def getState(self):
        return self.sim.get_state()

    def setState(self, values):
        currState = self.getState()

        newQpos = []
        for joint in self.joints:
            try:
                newvalue = values[joint]
                if newvalue != None:
                    try:
                        newvalue = float(newvalue)
                    except ValueError:
                        print("The joint values in values dictionary need to be either numeric or None")
                        return
                else:
                    newvalue=currState.qpos[self.joints.index(joint)]
            except KeyError:
                print(joint, " was not found from values dictionary")
            newQpos.append(newvalue)
        newState = mujoco_py.MjSimState(time=currState.time, qpos=newQpos,
                                        qvel=[0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.], act=None, udd_state={})
        self.sim.set_state(newState)
        return self.sim.get_state()                                    