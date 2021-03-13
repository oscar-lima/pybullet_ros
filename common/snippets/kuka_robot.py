import pybullet
import pybullet_data
import time
import os

pybullet.connect(pybullet.GUI)
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

robot = pybullet.loadURDF(os.path.expanduser('~') + '/repos_cloned/kuka_experimental/kuka_kr210_support/urdf/kr210l150.urdf', useFixedBase=1)
pybullet.setGravity(0, 0, -9.81)
plane = pybullet.loadURDF('plane.urdf')
pybullet.setRealTimeSimulation(0)
pybullet.setJointMotorControlArray(robot, range(6), pybullet.POSITION_CONTROL, targetPositions=[0.5] * 6)
# pybullet.setJointMotorControlArray(robot, range(6), pybullet.TORQUE_CONTROL, forces=[90000] * 6)
for _ in range(300):
    pybullet.stepSimulation()
    time.sleep(0.01)
