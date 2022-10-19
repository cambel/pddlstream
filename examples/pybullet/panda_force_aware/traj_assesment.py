#!/usr/bin/env python

from __future__ import print_function

from examples.pybullet.panda_force_aware.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_cfree_traj_grasp_pose_test, distance_fn

from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import Pose, Conf, get_ik_ir_gen, \
    get_stable_gen, get_grasp_gen, control_commands, get_torque_limits_not_exceded_test, \
    get_stable_gen_dumb, get_torque_limits_mock_test, get_ik_ir_gen_no_reconfig, hack_table_place,\
    get_ik_ir_gen_force_aware, get_torques_exceded_global, get_mass, METHOD, reset_torques_exceded_global,\
    get_ik_fn_force_aware
from examples.pybullet.utils.pybullet_tools.panda_utils import get_arm_joints, ARM_NAMES, get_group_joints, \
    get_group_conf, get_group_links, BI_PANDA_GROUPS, arm_from_arm, TARGET, PLATE_GRASP_LEFT_ARM, TIME_STEP, \
    set_joint_positions
from examples.pybullet.utils.pybullet_tools.utils import connect, get_pose, is_placement, disconnect, \
    get_joint_positions, HideOutput, LockRenderer, wait_for_user, get_max_limit, set_joint_positions_torque, set_point
from examples.pybullet.namo.stream import get_custom_limits

from pddlstream.algorithms.meta import create_parser, solve
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, from_test
from pddlstream.language.constants import Equal, And, print_solution, Exists, get_args, is_parameter, \
    get_parameter_name, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, Profiler
from pddlstream.language.function import FunctionInfo
from pddlstream.language.stream import StreamInfo, DEBUG

from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import apply_commands, State
from examples.pybullet.utils.pybullet_tools.utils import draw_base_limits, WorldSaver, has_gui, str_from_object, joint_from_name, is_pose_on_r, body_from_name, remove_fixed_constraint

from examples.pybullet.panda_force_aware.problems import PROBLEMS
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import Pose, Conf, get_ik_ir_gen, get_motion_gen, \
    get_stable_gen, get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State, FixObj
import time
import datetime
import pybullet as p
import csv

def run_traj_planner(problem, traj_planner, max_attempts=INF, max_time=INF):
  attempt = 0
  robot = problem.robot
  arm = "right"
  start_time = time.time()
  joints = get_arm_joints(problem.robot, "right")
  start_pose = get_joint_positions(problem.robot, joints)
  if problem.target is None or problem.end_grasp is None or problem.target_pose is None:
      return None
  path = None
  while attempt < max_attempts:
    if time.time() - start_time > max_time:
      return None
    set_joint_positions(robot, joints, start_pose)
    print("GRASP VALUE")
    print(problem.end_grasp.value)
    path = traj_planner(arm=arm, obj=problem.target, pose=problem.target_pose, grasp=problem.end_grasp)
    if path is not None:
        break

  return path

def execute_trajectory(traj_cmd, arm, target):
    trajectories = traj_cmd.commands
    open_gripper = GripperCommand(problem.robot, a, 0.5, teleport=teleport)
    detach = Detach(problem.robot, arm, target)
    if len(trajectories) == 2:
        print("reconfig present")
        [t1, t2] = c.commands
        new_commands = [t1, t2, detach, open_gripper, t2.reverse()]
    else:
        print("no reconfig")
        [t2] = c.commands
        new_commands = [t2, detach, open_gripper, t2.reverse()]
    with LockRenderer(lock=not args.enable):
          problem.remove_gripper()
    p.setRealTimeSimulation(True)
    time_step = None if args.teleport else TIME_STEP
    exec_time = time.time()
    state = State()
    time_step = None if args.teleport else TIME_STEP
    state = apply_commands(state, commands, time_step)

#   jointPos = get_joint_positions(problem.robot, jointNums)
#   set_joint_positions_torque(problem.robot, jointNums, jointPos)
    return time.time() - exec_time




def main(verbose=True):
    # TODO: could work just on postprocessing
    # TODO: try the other reachability database
    # TODO: option to only consider costs during local optimization

    parser = create_parser()
    parser.add_argument('-problem', default='packed_force_aware_transfer_traj_only', help='The name of the problem to solve')
    parser.add_argument('-loops', default=1, type=int, help='The number of itterations to run experiment')
    parser.add_argument('-n', '--number', default=1, type=int, help='The number of objects')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions')
    parser.add_argument('-deterministic', action='store_true', help='Uses a deterministic sampler')
    parser.add_argument('-optimal', action='store_true', help='Runs in an anytime mode')
    parser.add_argument('-t', '--max_time', default=250, type=int, help='The max time')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)

    problem_fn_from_name = {fn.__name__: fn for fn in PROBLEMS}
    problem_fn = problem_fn_from_name[args.problem]

    data_dir = '/home/liam/success_rate_data/'
    timestamp = str(datetime.datetime.now())
    datafile = data_dir + timestamp + "_" + args.problem + "_" + METHOD + '.csv'
    header = ["TotalTime", "ExecutionTime", "Solved", "TotalItems", "TorquesExceded", "MassPerObject", "Method"]
    with open(datafile, 'w') as file:
        writer = csv.writer(file)
        writer.writerow(header)

    global torques_exceded
    for _ in range(args.loops):
        reset_torques_exceded_global()
        connect(use_gui=True)
        print('connected to gui')
        with HideOutput():
            problem = problem_fn(num=args.number)
        print('problem found')
        draw_base_limits(problem.base_limits, color=(1, 0, 0))

        traj_planner = get_ik_fn_force_aware(problem)

        start_time = time.time()
        print("GRASP VALUE")
        print(problem.end_grasp.value)
        traj_cmd = run_traj_planner(problem, traj_planner, max_attempts=200, max_time=args.max_time)

        if traj_cmd is None:
            total_time = time.time() - start_time
            exec_time = -1
            items = args.number
            solved = False
            torques_exceded = get_torques_exceded_global()
            mass = get_mass(problem.movable[-1])
            data = [total_time, exec_time, solved, items, torques_exceded, mass, METHOD]
            with open(datafile, 'a') as file:
                writer = csv.writer(file)
                writer.writerow(data)
            disconnect()
            continue
        exec_time = execute_trajectory(traj_cmd, "left", problem.target)
        total_time = time.time() - start_time
        items = args.number
        solved = True
        mass = get_mass(problem.movable[0])
        torques_exceded = get_torques_exceded_global()
        data = [total_time, exec_time, solved, items, torques_exceded, mass, METHOD]
        with open(datafile, 'a') as file:
            writer = csv.writer(file)
            # writer.writerow(header)
            writer.writerow(data)
        disconnect()

if __name__ == "__main__":
    main()

