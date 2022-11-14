import numpy as np
import math 
from ..utils.pybullet_tools.rne import rne as RNE
from ..utils.pybullet_tools.panda_utils import TORQUE_TEST_LEFT_ARM
import csv
import os
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


###
# Method to calculate the centrifugal force for a single link
# parameters:
# m - mass of link
# r - norm of the vector from joint 1 to end of link assuming the arm is fully extended horizontally
# q - joint pose of joint 1
# qd - joint velocity of joint 1 
def F_cf(m, r, q, qd):
    # calculate r dot using r = ||r||cos(q)i + ||r||sin(q)j and taking the derivative
    velocity = np.array([-qd*r*math.sin(q), qd*r*math.cos(q), 0]) #assume the arm is straight out horizontal
    v_2 = velocity*velocity
    omega = np.array([0,0,qd]) 
    m_omega = np.array([0,0,-m*qd])
    r_vec = np.array([r*math.cos(q), r*math.sin(q), 0.0]) #vectorize r
    F = np.cross(m_omega, np.cross(omega, r_vec))
    return F

#####
# Function to calculate the coriolis force for a single link
# parameters:
# m - mass of link
# r - norm of the vector from joint 1 to end of link assuming the arm is fully extended horizontally
# q - joint pose of joint 1
# qd - joint velocity of joint 1 
def F_co(m, r, q, qd):
    omega2 = np.array([0,0,qd*2])
    rad = r
    velocity = np.array([-qd*rad*math.sin(q), qd*rad*math.cos(q), 0])
    return -m * np.cross(omega2, velocity)

###############
#  Calculate the total force required to accelerate a given link
# parameters:
# # parameters:
# m - mass of link
# r - norm of the vector from joint 1 to end of link assuming the arm is fully extended horizontally
# q - joint pose of joint 1
# qd - joint velocity of joint 1 
# qdd - joint acceleration of joint 1 
def F_link(m, r, q, qd, qdd):
    Fco  = F_co(m, r, q, qd)
    Fcf = F_cf(m, r, q, qd)
    d = r
    # calculate cartesian acceleration by taking the derivative of
    # rdot = -qd*sin(q)*||r|| i + qd * ||r|| * cos(q)j
    a = np.array([-d*(qd**2)*math.cos(q) - d*qdd*math.sin(q),-d*(qd**2)*math.sin(q) + d*qdd*math.cos(q), 0.0])
    # force required for acceration a for given link
    Ft = m*a
    F = Ft - Fcf
    F = F - Fco
    return F

# masses of each link
# second to last link is combination of hand and link5,
# joints are configured such that hand is folded in and is in line with link5 
# last link is combination of link6 and link7
ms = [0.646926, 3.228604, 3.587895, 1.225946 + 0.68, 1.666555+7.35522e-01]
# radius to the end of link or combination of links
rs = [0.3160*.70, 0.3160, 0.3840/4 + 0.3160, 0.3840*0.50 + 0.3160, 0.3840 + 0.3160]
#####
# Function to calculate the total torque required by joint 1
# Itterates through links, calculates the force for that link
# and calculates the torque by taking F x r
# Parameters:
# q - joint pose of joint 1
# qd - joint velocity of joint 1
# qdd - joint acceleration of joint 1 
def get_total_torque(q, qd, qdd):
    t_total = 0.0
    for link in range(len(rs)):
        Flink = F_link(ms[link], rs[link], q,qd,qdd)
        torque = rs[link]*math.cos(q) * Flink[1] + rs[link]*math.sin(q)*Flink[0]
        t_total += torque
    return t_total

"""
    <mass value="4.970684"/>
    <mass value="0.646926"/>
    <mass value="3.228604"/>
    <mass value="3.587895"/>
    <mass value="1.225946"/>
    <mass value="1.666555"/>
    <mass value="7.35522e-01"/>
    <mass value="0.0"/>
    <mass value="0.68"/>
    <mass value="0.0"/>"""

# rs = [0.3840 + 0.3160]
# ms = [0.646926+ 3.228604+ 3.587895 + 1.225946 + 0.68 + 1.666555+7.35522e-01]



joint_poses = TORQUE_TEST_LEFT_ARM[1:]
output_dir_name = '/home/liam/torque_calc_test_data/'
data_file_path = '/home/liam/success_rate_mass_data_random/ 2022-11-11 07:34:26.228705_packed_force_aware_transfer_base_9kg/2022-11-11 07:34:26.228705_trajectory_data_0.npz'

def load_npz_traj_data(file):
    data = np.load(file)
    qs = data['x']
    qds = data['y']
    qdds = data['z']
    return qs, qds, qdds

def calc_and_save_test(data_file_path, output_dir_name):
    name = data_file_path.split('/')[-1].split('.')[-2]
    print(name)
    file = output_dir_name + name + '_hand.csv'
    print(file)
    qs, qds, qdds = load_npz_traj_data(data_file_path)
    out_data = []
    for i in range(len(qs)):
        t = get_total_torque(qs[i][0],qds[i][0], qdds[i][0])
        out_data.append([qdds[i][0], qds[i][0], t])
    with open(file, 'w') as f:
        # create the csv writer
        writer = csv.writer(f)
        writer.writerows(out_data)

def calc_and_save_rne(data_file_path, output_dir_name):
    name = data_file_path.split('/')[-1].split('.')[-2]
    file = output_dir_name + name + '_rne.csv'
    qs, qds, qdds = load_npz_traj_data(data_file_path)
    out_data = []
    for i in range(len(qs)):
        q0 = qs[i][0]
        qd0 = qds[i][0]
        qdd0 = qdds[i][0]
        q = TORQUE_TEST_LEFT_ARM
        q[0] = q0
        q = np.array(q, dtype='float64')
        qd = np.array([qd0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype='float64')
        qdd = np.array([qdd0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype='float64')
        t = RNE(q, qd, qdd)[0]
        out_data.append([qdd0, qd0, t])
    with open(file, 'w') as f:
        # create the csv writer
        writer = csv.writer(f)
        writer.writerows(out_data)


def plot_torque_comp():
    rne_data = []
    test_data = []
    for file in os.listdir(output_dir_name):
        if 'rne' in file:
            rne_data = np.genfromtxt(output_dir_name+file, delimiter=',').T
        else:
            test_data = np.genfromtxt(output_dir_name+file, delimiter=',').T
    # for i in range(len(test_data[0])):
    #     print(test_data[1][i], rne_data[1][i], test_data[1][i]==rne_data[1][i])
    
    # plt.plot(rne_data[0], rne_data[2], label="RNE")
    # plt.plot(test_data[0], test_data[2], label="Hand Calculation")

    ax = plt.axes(projection='3d')
    ax.plot3D(rne_data[0], rne_data[1], rne_data[2], label="RNE")
    ax.plot3D(rne_data[0], rne_data[1], test_data[2], label="Hand Calculation")
    ax.set_xlabel('qdd')
    ax.set_ylabel('qd')
    ax.set_zlabel("Torque")
    
    plt.legend()
    # plt.xlabel('qdd')
    # plt.ylabel('Torque Output')
    plt.show()

def calculate_total_execution_time():
    qs, qds, qdds = load_npz_traj_data(data_file_path)
    total_time = 0
    all_times = []
    for i in range(len(qs)-1):
        times = []
        for j in range(len(qs[i])):
            times.append(abs(qs[i][j] - qs[i+1][j]))
            if times[-1] != 0:
                if qds[i][j] != 0:
                    times[-1] /= abs(qds[i][j])
                else:
                    times[-1] = 0
        total_time += max(times)
        all_times.append(max(times))
    print(float('inf') in all_times)
    return total_time, all_times 

if __name__ == '__main__':
    # calc_and_save_test(data_file_path, output_dir_name)
    # calc_and_save_rne(data_file_path, output_dir_name)
    # plot_torque_comp()
    total_time, all_times = calculate_total_execution_time()
    print(total_time, all_times)
    print(-float('inf') in all_times)
