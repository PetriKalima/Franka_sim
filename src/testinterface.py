from interface import Simpackage
import numpy as np

def main():
    newSim = Simpackage('../model/robots/samplefinal.xml')
    print(newSim.initState)
    print(newSim.sim.data.qfrc_bias)
    

    '''
    Joint values are radians, except for finger joints which use meters
    '''
    
    values = {"lumi_joint1": 0,
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
              }
              
    newstate = newSim.setState(values)
    while True:
        #values = updateValues(values, newSim)
        # newstate = newSim.setState(values)
        # print(newstate)
        newSim.step()
        print(newSim.sim.data.qfrc_applied)
        print(newSim.sim.data.ncon)
    '''
    values = {"lumi_joint1": 0.4,
              "lumi_joint2": None,
              "lumi_joint3": 0.3,
              "lumi_joint4": 0.2,
              "lumi_joint5": None,
              "lumi_joint6": 0.2,
              "lumi_joint7": 0.2,
              "lumi_finger_joint1": None,
              "lumi_finger_joint2": 0.05,
              "door_hinge": 0.5,
              "handle_core": 0.6 
              }
    newstate = newSim.setState(values)
    print(newstate)
    '''          
# def updateValues(values, newSim):
#     i = 0
#     for key in values.keys():
#         values[key] = -1 * newSim.sim.data.qfrc_bias[i]
#         i += 1
#     return values
          

main()   