import mujoco_py
import numpy as np


class Simpackage:

    def __init__(self, modelPath):
        self.model = mujoco_py.load_model_from_path(modelPath)
        self.sim = mujoco_py.MjSim(self.model)
        self.initState = self.sim.get_state()
        self.viewer = mujoco_py.MjViewer(self.sim)
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
        self.viewer.render()

    def getState(self):
        state = self.sim.get_state()
        return np.array([state.qpos, state.qvel])

    def setState(self, values):
        currState = self.sim.get_state()

        newQpos = []
        for joint in self.joints:
            try:
                newvalue = values[joint]
                try:
                    newvalue = float(newvalue)
                    self.sim.data.ctrl[self.joints.index(joint)] = newvalue
                except ValueError:
                    print("given value must be numeric")    
            except KeyError:
                print(joint, " was not found")
        
        return self.getState()            

    def step(self):
        self.sim.step()
        self.view()

    def reset(self):
        self.sim.set_state(self.initState)

'''
TODO
-Change step-function to imitate hockeypuck_env (action given straight to step)
-Counter gravity with self.sim.data.qfrc_bias
'''        

    
    

