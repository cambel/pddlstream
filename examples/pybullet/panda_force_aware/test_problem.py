from examples.pybullet.panda_force_aware.problems import PROBLEMS
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, get_max_limit, set_joint_positions_torque, set_point
import pybullet as p
import time
from ..utils.pybullet_tools.utils import get_pose, get_name

def test_packed_transfer():
  problem = PROBLEMS[1](num=1)

def test_packed_transfer_traj_only():
  problem = PROBLEMS[3](num=1)

def test_hiro():
  problem = PROBLEMS[4](num=1)

def main():
  connect(use_gui=True)
  test_hiro()
  get_name(3)
  p.setRealTimeSimulation(True)
  wait_for_user()
  disconnect()


if __name__ == '__main__':
  main()