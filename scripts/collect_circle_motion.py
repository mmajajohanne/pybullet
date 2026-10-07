import pybullet as p
import pybullet_data
import numpy as np
import time

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

p.resetDebugVisualizerCamera(
    cameraDistance=1.8, cameraYaw=50, cameraPitch=-20, cameraTargetPosition=[0, 0, 0.8]
)

dt = 1.0 / 240.0
p.setTimeStep(dt)

TOOL_LINK = 6
NUM_JOINTS = 7

# sirkel-parametere
CENTER = np.array([0.5, 0.0, 0.8])  # midtpunkt for sirkel i verden (x, y, z)
RADIUS = 0.2  # 20 cm radius
PERIOD = 4.0  # sekunder per full runde
DURATION = 8.0  # kjør i 8 sekunder (= 2 runder)
num_steps = int(DURATION / dt)

# datalagring
timestamps, positions, orientations = [], [], []
linear_velocities, angular_velocities = [], []

# lagre også ønsket posisjon for sammenligning
commanded_positions = []

for step in range(num_steps):
    t = step * dt

    # regn ut ønsket posisjon på sirkelen
    angle = 2 * np.pi * t / PERIOD
    target_x = CENTER[0] + RADIUS * np.cos(angle)
    target_y = CENTER[1] + RADIUS * np.sin(angle)
    target_z = CENTER[2]
    target_pos = [target_x, target_y, target_z]

    # regn ut ledd-vinkler som gir denne posisjonen (invers kinematikk)
    joint_angles = p.calculateInverseKinematics(robot, TOOL_LINK, target_pos)

    # kommander hvert ledd til sin målposisjon
    for j in range(NUM_JOINTS):
        p.setJointMotorControl2(
            robot, j, p.POSITION_CONTROL, targetPosition=joint_angles[j]
        )

    p.stepSimulation()

    # hent ut faktisk tool-tilstand
    link_state = p.getLinkState(robot, TOOL_LINK, computeLinkVelocity=1)
    timestamps.append(t)
    positions.append(link_state[0])
    orientations.append(link_state[1])
    linear_velocities.append(link_state[6])
    angular_velocities.append(link_state[7])
    commanded_positions.append(target_pos)

    time.sleep(dt)

p.disconnect()

# konverter og skriv ut
timestamps = np.array(timestamps)
positions = np.array(positions)
commanded_positions = np.array(commanded_positions)
linear_velocities = np.array(linear_velocities)
angular_velocities = np.array(angular_velocities)

# sjekk hvor godt den faktisk følger sirkelen
tracking_error = np.linalg.norm(positions - commanded_positions, axis=1)
print(f"Gjennomsnittlig tracking-feil: {np.mean(tracking_error)*1000:.2f} mm")
print(f"Maks tracking-feil: {np.max(tracking_error)*1000:.2f} mm")
print(
    f"Maks lineær hastighet: {np.max(np.linalg.norm(linear_velocities, axis=1)):.3f} m/s"
)
print(
    f"Maks angulær hastighet: {np.max(np.linalg.norm(angular_velocities, axis=1)):.3f} rad/s"
)

# teoretisk hastighet for sirkelbevegelse: v = 2pir/T
theoretical_speed = 2 * np.pi * RADIUS / PERIOD
print(f"Teoretisk sirkelhastighet: {theoretical_speed:.3f} m/s")

np.savez(
    "data/circle_motion.npz",
    t=timestamps,
    pos=positions,
    commanded=commanded_positions,
    orn=np.array(orientations),
    lin_vel=linear_velocities,
    ang_vel=angular_velocities,
)
print("Lagret circle_motion.npz")
