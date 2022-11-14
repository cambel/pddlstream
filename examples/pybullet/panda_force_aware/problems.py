from __future__ import print_function

import numpy as np
import math

from examples.pybullet.utils.pybullet_tools.bi_panda_problems import create_bi_panda, create_short_table, Problem, create_table, create_panda
from examples.pybullet.utils.pybullet_tools.panda_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, \
    arm_conf, REST_LEFT_ARM, close_arm, set_group_conf, STRAIGHT_LEFT_ARM, get_extended_conf, get_group_links, PLATE_GRASP_LEFT_ARM,\
    get_max_limit, get_gripper_joints, TOP_HOLDING_LEFT_ARM_CENTERED, TOP_HOLDING_TRAJ_START, get_top_cylinder_grasps, TORQUE_TEST_LEFT_ARM
from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY, create_cylinder, enable_gravity, link_from_name, get_link_pose, \
    Pose, set_joint_position, TRAY_URDF, set_pose, add_fixed_constraint, COKE_URDF, get_unit_vector, multiply, unit_quat,\
    get_pose, HIRO_TABLE_2, HIRO_TABLE_1, WALL_URDF, quat_from_euler
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import set_joint_force_limits
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import Pose as PrimPose
import pybullet as p
from examples.pybullet.utils.pybullet_tools.panda_primitives_v2 import set_joint_force_limits, Attach, control_commands,\
    Grasp, GripperCommand, get_top_grasps, GRASP_LENGTH, APPROACH_DISTANCE, set_joint_positions_torque, get_arm_joints
from random import uniform

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True
#####################################################
def packed_force_aware_transfer_HIRO(arm='right', grasp_type='top', num=1, dist=0.5, high_angle=math.pi/4, low_angle = -math.pi/4):
    # TODO: packing problem where you have to place in one direction
    print('in packed')
    base_extent = 5.0
    X_DIST = dist
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    plate_width = 0.2
    #plate_width = 0.28
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.04)
    plate_height = 0.005

    initial_conf = get_carry_conf(arm, grasp_type)
    add_data_path()
    floor = load_pybullet("plane.urdf")
    set_point(floor, (0,0,-1))
    panda = create_panda()
    # set_point(panda,point=Point(0,0, 0.1))
    set_joint_force_limits(panda, arm)
    set_arm_conf(panda, arm, initial_conf)
    open_arm(panda, arm)
    # set_point(panda, (0,0,0.4))
    table = load_pybullet(HIRO_TABLE_1)
    set_point(table, (-0.2994,0,-0.5131))
    add_fixed_constraint(table, floor)
    table2 = load_pybullet(HIRO_TABLE_2)
    set_point(table2, (0.6218, 0,-0.53645))
    add_fixed_constraint(table2, floor)
    wall = load_pybullet(WALL_URDF)
    set_pose(wall, ((-0.7366, 0,0),quat_from_euler((0,0,0))))
    add_fixed_constraint(wall, floor)
    # add_fixed_constraint(table, table2)


    # set_point(table, point=Point(0.55,0, 0))
    # set_point(table2, point=Point(-0.45,0, 0))
    start_plate = create_box(.5, .9, .01, color=GREEN)
    plate_z = stable_z(start_plate, table)
    set_point(start_plate, (.5, 0, plate_z))
    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table)
    set_point(plate, Point(x=0, y=-.45, z=plate_z ))
    add_fixed_constraint(plate, table)
    surfaces = [table, plate]
    pick_area = table
    place_area = table2

    blocks = [load_pybullet(COKE_URDF) for _ in range(num)]
    initial_surfaces = {block: start_plate for block in blocks}

    min_distances = {block: 0.02 for block in blocks}
    sample_placements(initial_surfaces)
    start_dist = get_pose(blocks[0])
    theta = uniform(low_angle, high_angle)
    new_x = X_DIST * math.cos(theta)
    new_y = X_DIST * math.sin(theta)
    obj_z = stable_z(blocks[0], start_plate)
    set_point(blocks[0], (new_x, new_y, obj_z))
    enable_gravity()
    return Problem(robot=panda, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                #    goal_holding=[(arm, plate)],
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits, dist = X_DIST)



#######################################################
def packed_force_aware_transfer(arm='right', grasp_type='top', num=1, dist=0.5):
    # TODO: packing problem where you have to place in one direction
    print('in packed')
    base_extent = 5.0
    X_DIST = dist
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    plate_width = 0.2
    #plate_width = 0.28
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.04)
    plate_height = 0.005

    # initial_conf = get_carry_conf(arm, grasp_type)
    initial_conf = TORQUE_TEST_LEFT_ARM
    add_data_path()
    floor = load_pybullet("plane.urdf")
    set_point(floor, (0,0,-.001))
    panda = create_panda()
    # set_point(panda,point=Point(0,0, 0.1))
    set_joint_force_limits(panda, arm)
    set_arm_conf(panda, arm, initial_conf)
    open_arm(panda, arm)
    # set_point(panda, (0,0,0.4))
    table = create_table(length=.8, height=0.1, width = 0.6)
    table2 = create_table(length=0.35, height=0.1, width = 0.3)

    set_point(table, point=Point(0.55,0, 0))
    set_point(table2, point=Point(-0.45,0, 0))

    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table2)
    set_point(plate, Point(x=-0.45, z=plate_z))
    surfaces = [table, plate]
    pick_area = table
    place_area = table2

    blocks = [load_pybullet(COKE_URDF) for _ in range(num)]
    initial_surfaces = {block: table for block in blocks}

    min_distances = {block: 0.02 for block in blocks}
    sample_placements(initial_surfaces)
    start_dist = get_pose(blocks[0])
    theta = math.atan2(start_dist[0][1], start_dist[0][0])
    new_x = X_DIST * math.sin(theta)
    new_y = X_DIST * math.cos(theta)
    obj_z = stable_z(blocks[0], table)
    set_point(blocks[0], (new_x, new_y, obj_z))
    enable_gravity()
    return Problem(robot=panda, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                #    goal_holding=[(arm, plate)],
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits, dist = X_DIST)

def packed_force_aware_transfer_traj_only(arm='right', grasp_type='top', num=1):
    # TODO: packing problem where you have to place in one direction
    print('in packed')
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))

    X_OFFSET = 0.35

    plate_width = 0.2
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.04)
    plate_height = 0.005

    initial_conf = TOP_HOLDING_TRAJ_START
    add_data_path()
    floor = load_pybullet("plane.urdf")
    panda = create_panda()
    set_point(panda,point=Point(0,0, 0.1))
    set_joint_force_limits(panda, arm)
    set_arm_conf(panda, arm, initial_conf)
    open_arm(panda, arm)

    table = create_table(length=0.35, height=0.4, width = 0.3)
    table2 = create_table(length=0.35, height=0.4, width = 0.3)
    r_left_finger_joint = joint_from_name(panda, 'r_panda_finger_joint1')

    set_point(table, point=Point(X_OFFSET,0, 0.02))
    set_point(table2, point=Point(-X_OFFSET,0, 0.02))

    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table2)
    set_point(plate, Point(x=-X_OFFSET, z=plate_z))
    surfaces = [table, plate]
    pick_area = table
    place_area = table2

    block = load_pybullet(COKE_URDF)
    blocks = [block]
    block_z = stable_z(block, table)
    start_point = (X_OFFSET-0.042, 0.0, block_z)

    target_z = stable_z(block, plate)
    target_point = (-X_OFFSET, 0.0, target_z)

    set_point(block, target_point)
    target_pose = get_pose(block)
    target_pose = PrimPose(block, target_pose)
    g = get_top_grasps(block, grasp_length=GRASP_LENGTH)[0]

    set_point(block, start_point)
    close_arm(panda, arm)

    start_pose = get_pose(block)
    grasp = Grasp('top', block, start_pose, [], [])
    attach = Attach(panda, arm, grasp, block)#Attach(bi_panda, arm, grasp, tray)
    control_commands([attach])
    set_joint_positions_torque(panda, get_arm_joints(panda, arm), TOP_HOLDING_TRAJ_START)
    approach_vector = APPROACH_DISTANCE*get_unit_vector([1, 0, 0])
    target_grasp = Grasp(grasp_type="top", body=block, value=g, approach=multiply((approach_vector, unit_quat()), g), carry=TOP_HOLDING_TRAJ_START)

    enable_gravity()



    return Problem(robot=panda, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                #    goal_holding=[(arm, plate)],
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits, target=block,\
                    target_pose=target_pose, end_grasp=target_grasp)


#######################################################

def packed_force_aware(arm='right', grasp_type='top', num=2):
    # TODO: packing problem where you have to place in one direction
    print('in packed')
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.04
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    plate_width = 0.2
    #plate_width = 0.28
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.04)
    plate_height = 0.005

    initial_conf = get_carry_conf(arm, grasp_type)
    add_data_path()
    floor = load_pybullet("plane.urdf")
    panda = create_panda()
    set_point(panda,point=Point(0,0, 0.1))
    set_joint_force_limits(panda, arm)
    set_arm_conf(panda, arm, initial_conf)
    open_arm(panda, arm)

    table = create_table(length=0.35, height=0.4, width = 0.3)
    r_left_finger_joint = joint_from_name(panda, 'r_panda_finger_joint1')
    #right finger joint
    # r_right_finger_joint = joint_from_name(panda, 'r_panda_finger_joint2')
    # set_joint_position(panda, r_right_finger_joint,block_width)
    # set_joint_position(panda, r_left_finger_joint, block_width)
    set_point(table, point=Point(0.5,0, 0.02))
    # add_fixed_constraint(table, floor)
    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table)
    set_point(plate, Point(x=0.5, z=plate_z))
    surfaces = [table, plate]

    blocks = [load_pybullet(COKE_URDF) for _ in range(num)]
    initial_surfaces = {block: table for block in blocks}

    min_distances = {block: 0.02 for block in blocks}
    sample_placements(initial_surfaces)
    set_point(blocks[0], (0.49569211602211, -0.0283925175666809, 0.4259999990463257))
    enable_gravity()
    return Problem(robot=panda, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                #    goal_holding=[(arm, plate)],
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits)

#######################################################

def blocked(arm='left', grasp_type='side', num=1):
    x_extent = 10.0

    base_limits = (-x_extent/2.*np.ones(2), x_extent/2.*np.ones(2))
    block_width = 0.04
    #block_height = 0.1
    block_height = 2*block_width
    #block_height = 0.2
    plate_height = 0.001
    table_x = (x_extent - 1) / 2.

    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    bi_panda = create_bi_panda()
    set_arm_conf(bi_panda, arm, initial_conf)
    open_arm(bi_panda, arm)
    set_arm_conf(bi_panda, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(bi_panda, other_arm)
    set_group_conf(bi_panda, 'base', [x_extent/4, 0, 0]) # Be careful to not set the bi_panda's pose

    #table3 = create_table()
    #set_point(table3, Point(x=0, y=0))

    plate = create_box(0.6, 0.6, plate_height, color=GREEN)
    x, y, _ = get_point(table1)
    plate_z = stable_z(plate, table1)
    set_point(plate, Point(x=x, y=y-0.3, z=plate_z))
    #surfaces = [table1, table2, table3, plate]
    surfaces = [table1, table2, plate]

    green1 = create_box(block_width, block_width, block_height, color=BLUE)
    green1_z = stable_z(green1, table1)
    set_point(green1, Point(x=x, y=y+0.3, z=green1_z))
    # TODO: can consider a fixed wall here instead

    spacing = 0.15

    #red_directions = [(-1, 0), (+1, 0), (0, -1), (0, +1)]
    red_directions = [(-1, 0)]
    #red_directions = []
    red_bodies = []
    for red_direction in red_directions:
        red = create_box(block_width, block_width, block_height, color=RED)
        red_bodies.append(red)
        x, y = get_point(green1)[:2] + spacing*np.array(red_direction)
        z = stable_z(red, table1)
        set_point(red, Point(x=x, y=y, z=z))

    wall1 = create_box(0.01, 2*spacing, block_height, color=GREY)
    wall2 = create_box(spacing, 0.01, block_height, color=GREY)
    wall3 = create_box(spacing, 0.01, block_height, color=GREY)
    z = stable_z(wall1, table1)
    x, y = get_point(green1)[:2]
    set_point(wall1, Point(x=x+spacing, y=y, z=z))
    set_point(wall2, Point(x=x+spacing/2, y=y+spacing, z=z))
    set_point(wall3, Point(x=x+spacing/2, y=y-spacing, z=z))

    green_bodies = [create_box(block_width, block_width, block_height, color=BLUE) for _ in range(num)]
    body_types = [(b, 'green') for b in [green1] + green_bodies] #  + [(table1, 'sink')]

    movable = [green1] + green_bodies + red_bodies
    initial_surfaces = {block: table2 for block in green_bodies}
    sample_placements(initial_surfaces)

    return Problem(robot=bi_panda, movable=movable, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                   #sinks=[table1],
                   #goal_holding=[(arm, '?green')],
                   #goal_cleaned=['?green'],
                   goal_on=[('?green', plate)],
                   body_types=body_types, base_limits=base_limits, costs=True)

#######################################################


PROBLEMS = [
    packed_force_aware,
    packed_force_aware_transfer,
    blocked,
    packed_force_aware_transfer_traj_only,
    packed_force_aware_transfer_HIRO
]