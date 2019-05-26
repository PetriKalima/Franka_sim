import mujoco_py
import numpy as np
from mujoco_py import functions


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
                    self.sim.data.qpos[self.joints.index(joint)] = newvalue
                    self.sim.data.qvel[self.joints.index(joint)] = 0
                except ValueError:
                    print("given value must be numeric")    
            except KeyError:
                print(joint, " was not found")
        
        return self.getState()         

    def step(self, action):
        rne = np.ndarray(self.sim.model.nv)
        joint_idx = [self.sim.model.get_joint_qpos_addr(j) for j in self.joints]

        # Compute forces required for gravity, Coriolis, etc compensation
        # False indicates that we set the desired acceleration to zero,
        # (ignore M*desired_acc); the results should be the same as
        # if we directly read qfrc_bias
        functions.mj_rne(self.sim.model, self.sim.data, False, rne)

        # Set them as applied forces to each joint
        # There are two ways of doing this:
        # 1. By adding it to actuators (in this case, the actuators
        # take the gravity force on themselves, so if there are any force limits
        # imposed on the actuators, gravity compensation will also be limited
        self.sim.data.ctrl[joint_idx] = rne[joint_idx] + action

        # 2. By directly adding it as a compensation force -- it looks like
        # a less realistic scenario, with the force kind of "magically" appearing
        # out of nowhere (commented out so that we don't compensate twice)
        #self.sim.data.qfrc_applied[joint_idx] = rne[joint_idx]

        # Step, render, etc
        self.sim.step()
        self.view()

    def reset(self):
        self.sim.set_state(self.initState)

'''
TODO
-Change step-function to imitate hockeypuck_env (action given straight to step)
-Counter gravity with self.sim.data.qfrc_bias
'''        

    
    

