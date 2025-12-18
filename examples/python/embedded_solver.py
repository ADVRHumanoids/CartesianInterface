#!/usr/bin/env python3

"""
Example of embedding the CartesIO solver in a Python script which is 
completely independent from ROS and standalone. 
The required input is just a URDF file of the robot.
"""

import math
import os
from time import sleep

from cartesian_interface.pyci_all import *
from xbot2_interface.pyaffine3 import Affine3
from xbot2_interface import pyxbot2_interface as xb 

# load the URDF and SRDF file
urdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../urdf/coman.urdf"
urdf = open(urdf_path, "r").read()
srdf_path = os.path.dirname(os.path.abspath(__file__)) + "/../srdf/coman.srdf"  
srdf = open(srdf_path, "r").read()

# params
DT = 0.01
LOG_PATH = '/tmp'
TRJ_RADIUS = 0.1  # m
TRJ_FREQ_HZ = 0.1  # Hz

# create the model interface
model = xb.ModelInterface2(urdf, srdf, 'pin')

# load the ik problem
ik_problem_path = os.path.dirname(os.path.abspath(__file__)) + "/../configs/coman_stack.yaml"
ik_problem = open(ik_problem_path, "r").read()

# reset model position to homing
model.q = model.getRobotState('home')
model.update()

# create the solver
solver = pyci.CartesianInterface.MakeInstance(
    solver='OpenSot',
    problem=ik_problem,
    model=model,
    dt=DT,
    log_path=LOG_PATH
)

# get the left hand task
lh_task = solver.getTask('left_hand')
lh_ee_name = lh_task.getDistalLink()
lh_base_name = lh_task.getBaseLink()
print(f'+++ Left hand ee link: {lh_ee_name}, base link: {lh_base_name}')

# initial ee pose
T_0 = model.getPose(lh_ee_name, lh_base_name)
print(f'+++ Initial left hand ee pose:\n{T_0}')

# ik loop
time = 0
while True:
    
    # define desired pose
    Tdes = T_0.copy()
    Tdes.translation[1] += TRJ_RADIUS * math.sin(2 * math.pi * TRJ_FREQ_HZ * time)
    Tdes.translation[2] += TRJ_RADIUS * (1 - math.cos(2 * math.pi * TRJ_FREQ_HZ * time))/2.

    # set desired pose
    lh_task.setPoseReference(Tdes)

    # solve
    if not solver.update(time, DT):
        print("!!! Solver failed!")
        break

    # integrate solution
    model.q = model.sum(model.q, model.v * DT)
    model.update()

    # print tracking error
    T = model.getPose(lh_ee_name, lh_base_name)
    error = T * Tdes.inverse()
    print(f'Time: {time:.2f} s, Error matrix:\n{error}')

    # increment time
    time += DT

    # sync 
    sleep(DT)



