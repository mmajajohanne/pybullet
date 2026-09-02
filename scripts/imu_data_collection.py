import pybullet as p
import pybullet_data
import numpy as np
import time

# oppsett
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

# simuleringsparametere
dt = 1.0 / 240.0  # PyBullet-standard: 240 Hz
p.setTimeStep(dt)

TOOL_LINK = 6  # siste linken = tool
DURATION = 5.0  # sekunder
num_steps = int(DURATION / dt)

# datalagring
timestamps = []
positions = []  # toolens posisjon i verden
orientations = []  # som kvaternion (x, y, z, w)
linear_velocities = []
angular_velocities = []

# kjør en enkel, jevn bevegelse: sinusformet rotasjon i ledd 1 og 3
for step in range(num_steps):
    t = step * dt  # teller fra 0 til 1199

    # ledd-vinkler
    """
    target_j1 = 0.5 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz, amplitude 0.5 rad
    target_j3 = 0.3 * np.sin(2 * np.pi * 0.3 * t)  # 0.3 Hz, amplitude 0.3 rad

    p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, targetPosition=target_j1)
    p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, targetPosition=target_j3)
    """
    # Beveg flere ledd for mer synlig og interessant bevegelse
    target_j1 = 0.8 * np.sin(2 * np.pi * 0.3 * t)
    target_j2 = 0.6 * np.sin(2 * np.pi * 0.4 * t) + 0.5  # offset så armen bøyer seg
    target_j3 = 0.5 * np.sin(2 * np.pi * 0.5 * t)
    target_j4 = 0.7 * np.sin(2 * np.pi * 0.6 * t) - 0.8  # offset så albuen bøyer seg

    p.setJointMotorControl2(robot, 1, p.POSITION_CONTROL, targetPosition=target_j1)
    p.setJointMotorControl2(robot, 2, p.POSITION_CONTROL, targetPosition=target_j2)
    p.setJointMotorControl2(robot, 3, p.POSITION_CONTROL, targetPosition=target_j3)
    p.setJointMotorControl2(robot, 5, p.POSITION_CONTROL, targetPosition=target_j4)

    p.stepSimulation()

    # Hent ut "sann" tilstand for tool-flensen
    link_state = p.getLinkState(robot, TOOL_LINK, computeLinkVelocity=1)
    pos = link_state[0]  # verdens-posisjon
    orn = link_state[1]  # verdens-orientering (kvaternion)
    lin_vel = link_state[6]  # lineær hastighet i verden
    ang_vel = link_state[7]  # angulær hastighet i verden

    timestamps.append(t)
    positions.append(pos)
    orientations.append(orn)
    linear_velocities.append(lin_vel)
    angular_velocities.append(ang_vel)

    time.sleep(dt)  # kjør i sanntid så man kan se det

p.disconnect()

# Konverter til numpy-arrays for videre analyse
timestamps = np.array(timestamps)
positions = np.array(positions)
orientations = np.array(orientations)
linear_velocities = np.array(linear_velocities)
angular_velocities = np.array(angular_velocities)

print(f"Samlet {len(timestamps)} datapunkter over {DURATION} sekunder")
print(f"Posisjonsform: {positions.shape}")
print(f"Første posisjon: {positions[0]}")
print(f"Siste posisjon: {positions[-1]}")
print(
    f"Maks lineær hastighet: {np.max(np.linalg.norm(linear_velocities, axis=1)):.3f} m/s"
)
print(
    f"Maks angulær hastighet: {np.max(np.linalg.norm(angular_velocities, axis=1)):.3f} rad/s"
)

# Lagre til disk for senere bruk
np.savez(
    "ground_truth.npz",
    t=timestamps,
    pos=positions,
    orn=orientations,
    lin_vel=linear_velocities,
    ang_vel=angular_velocities,
)
print("Lagret ground_truth.npz")
