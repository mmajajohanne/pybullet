import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# skriv ut info om alle ledd
num_joints = p.getNumJoints(robot)
print(f"Antall ledd: {num_joints}")
print("-" * 60)
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode()
    joint_type = info[2]
    link_name = info[12].decode()
    print(f"Ledd {i}: navn={joint_name}, type={joint_type}, link={link_name}")

# Hold vinduet åpent
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

p.disconnect()
