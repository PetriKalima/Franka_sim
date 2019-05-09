from interface import Simpackage

def main():
    newSim = Simpackage('../model/robots/samplefinal.xml')
    print(newSim.initState)
    values = {"lumi_joint1": 0.2,
              "lumi_joint2": 0.2,
              "lumi_joint3": 0.2,
              "lumi_joint4": 0.2,
              "lumi_joint5": 0.2,
              "lumi_joint6": 0.2,
              "lumi_joint7": 0.2,
              "lumi_finger_joint1": 0.001,
              "lumi_finger_joint2": 0.001,
              "door_hinge": 0.5,
              "handle_core": 0.6 
              }
    newstate = newSim.setState(values)
    print(newstate)
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
    print(newSim.getState())
    newSim.view()

main()   