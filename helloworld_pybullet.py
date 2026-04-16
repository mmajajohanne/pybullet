import pybullet as p
import pybullet_data
import time

# Koble til simulator med grafikk
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Last inn gulv og en robotarm (KUKA følger med PyBullet)
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# La simulatoren kjøre og beveg ledd 1 i en sinus
for i in range(10000):
    angle = 0.8 * (i / 240)  # sakte bevegelse
    p.setJointMotorControl2(robot, jointIndex=1, 
                             controlMode=p.POSITION_CONTROL, 
                             targetPosition=angle)
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()