import pybullet as p
import pybullet_data
import numpy as np
import time

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Bundled pybullet_data table.urdf is ~0.63 m tall, which puts anything on it
# right at the edge of (or beyond) the Panda's ~0.855 m reach once you add a
# hover height on top. Use a low table (box) placed close to the base instead,
# which keeps the grasp comfortably inside the arm's workspace.
TABLE_HEIGHT = 0.3
TABLE_HALF_EXTENTS = [0.3, 0.3, TABLE_HEIGHT / 2]
TABLE_POS = [0.4, 0, TABLE_HEIGHT / 2]

table_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=TABLE_HALF_EXTENTS)
table_vis = p.createVisualShape(
    p.GEOM_BOX, halfExtents=TABLE_HALF_EXTENTS, rgbaColor=[0.55, 0.4, 0.25, 1]
)
table = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=table_col,
    baseVisualShapeIndex=table_vis,
    basePosition=TABLE_POS,
)

CUBE_START = [0.4, 0, TABLE_HEIGHT + 0.025]
DROP_POS = [0.4, 0.25, TABLE_HEIGHT + 0.025]
cube = p.loadURDF("cube_small.urdf", CUBE_START)
p.changeVisualShape(cube, -1, rgbaColor=[0.9, 0.2, 0.2, 1])
p.changeDynamics(cube, -1, lateralFriction=1.0)
for finger_link in (9, 10):
    p.changeDynamics(robot, finger_link, lateralFriction=1.0)

p.resetDebugVisualizerCamera(
    cameraDistance=1.1, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0.4, 0, 0.4]
)

dt = 1.0 / 240.0
p.setTimeStep(dt)

EE_LINK = 11
ARM_JOINTS = list(range(7))
FINGER_JOINTS = [9, 10]
FINGER_OPEN = 0.04
FINGER_CLOSE = 0.012
FINGER_FORCE = 40

HOVER_HEIGHT = 0.25
GRASP_HEIGHT = CUBE_START[2]

# gripper pointing straight down for a top-down grasp
DOWN_ORN = p.getQuaternionFromEuler([np.pi, 0, 0])

# PyBullet's IK solver seeds from the arm's *current* joint state and doesn't
# enforce joint limits on its own. panda_joint4 only allows negative angles
# (-3.14 to 0); starting IK from the all-zero default pose makes the solver
# converge toward a positive (out-of-range) joint4, which the physics engine
# then clamps, sending the arm to the wrong place entirely. Warm-starting from
# a sane "ready" pose keeps every subsequent IK solve in the feasible basin.
READY_POSE = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for j, angle in zip(ARM_JOINTS, READY_POSE):
    p.resetJointState(robot, j, angle)


def move_ee_to(pos, orn, steps):
    # Position control with the default (huge) max force snaps the whole arm
    # toward the IK target in essentially one step, which jerks hard enough to
    # shake a grasped cube loose. Interpolating the joint targets smoothly over
    # `steps` keeps joint velocities (and the resulting end-effector
    # accelerations) bounded instead.
    target_angles = p.calculateInverseKinematics(
        robot, EE_LINK, pos, orn, maxNumIterations=200, residualThreshold=1e-5
    )
    start_angles = [p.getJointState(robot, j)[0] for j in ARM_JOINTS]
    for step in range(steps):
        alpha = (step + 1) / steps
        for idx, j in enumerate(ARM_JOINTS):
            waypoint = start_angles[idx] + alpha * (target_angles[idx] - start_angles[idx])
            p.setJointMotorControl2(robot, j, p.POSITION_CONTROL, targetPosition=waypoint)
        p.stepSimulation()
        time.sleep(dt)


def set_fingers(target, force=FINGER_FORCE):
    for j in FINGER_JOINTS:
        p.setJointMotorControl2(
            robot, j, p.POSITION_CONTROL, targetPosition=target, force=force
        )


def wait(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(dt)


# datalagring
timestamps, cube_positions = [], []
step_counter = [0]


def log():
    step_counter[0] += 1
    timestamps.append(step_counter[0] * dt)
    cube_positions.append(p.getBasePositionAndOrientation(cube)[0])


# 1. hover over cube, fingers open
set_fingers(FINGER_OPEN)
hover_pos = [CUBE_START[0], CUBE_START[1], CUBE_START[2] + HOVER_HEIGHT]
move_ee_to(hover_pos, DOWN_ORN, 120)
log()

# 2. descend to grasp height
grasp_pos = [CUBE_START[0], CUBE_START[1], GRASP_HEIGHT]
move_ee_to(grasp_pos, DOWN_ORN, 120)
log()

# 3. close fingers and let contact settle
set_fingers(FINGER_CLOSE)
wait(120)
log()

# 4. lift straight up
lift_pos = [CUBE_START[0], CUBE_START[1], CUBE_START[2] + HOVER_HEIGHT]
move_ee_to(lift_pos, DOWN_ORN, 120)
log()
peak_height = p.getBasePositionAndOrientation(cube)[0][2]

# 5. translate to drop location
drop_hover = [DROP_POS[0], DROP_POS[1], DROP_POS[2] + HOVER_HEIGHT]
move_ee_to(drop_hover, DOWN_ORN, 150)
log()

# 6. descend and release
move_ee_to(DROP_POS, DOWN_ORN, 120)
log()
set_fingers(FINGER_OPEN)
wait(60)
log()

# 7. retract
retract_pos = [DROP_POS[0], DROP_POS[1], DROP_POS[2] + HOVER_HEIGHT]
move_ee_to(retract_pos, DOWN_ORN, 120)
log()

p.disconnect()

timestamps = np.array(timestamps)
cube_positions = np.array(cube_positions)

final_pos = cube_positions[-1]
drop_error = np.linalg.norm(np.array(final_pos[:2]) - np.array(DROP_POS[:2]))

print(f"Starthøyde kube: {CUBE_START[2]:.3f} m")
print(f"Makshøyde kube under løft: {peak_height:.3f} m")
print(f"Sluttposisjon kube: {final_pos}")
print(f"Avstand sluttposisjon -> mål (xy): {drop_error*1000:.1f} mm")

grasp_ok = peak_height > CUBE_START[2] + 0.1
place_ok = drop_error < 0.05
print(f"Grep lyktes (kube hevet): {grasp_ok}")
print(f"Plassering lyktes (innen 5 cm av mål): {place_ok}")

np.savez(
    "data/pick_and_place.npz",
    t=timestamps,
    cube_pos=cube_positions,
    cube_start=np.array(CUBE_START),
    drop_target=np.array(DROP_POS),
)
print("Lagret pick_and_place.npz")
