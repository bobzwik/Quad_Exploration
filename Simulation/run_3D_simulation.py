# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import cProfile

from trajectory import Trajectory
from potentialField import PotField
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config

def quad_sim(t, Ts, quad, ctrl, wind, traj, potfld):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts

    # Trajectory for Desired States 
    # ---------------------------
    sDes = traj.desiredState(t, Ts, quad)

    # Potential Field Influence 
    # ---------------------------     
    potfld.isWithinRange(quad)
    potfld.isWithinField(quad)
    potfld.rep_force(quad)

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(traj, quad, sDes, potfld, Ts)

    return t
    

def main():
    start_time = time.time()

    # Simulation Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005
    Tf = 70
    ifsave = 0

    # Choose trajectory settings
    # --------------------------- 
    ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
    trajSelect = np.zeros(3)

    # Select Control Type             (0: xyz_pos,       1: xy_vel_z_pos,            2: xyz_vel)
    ctrlType = ctrlOptions[0]   
    # Select Position Trajectory Type (0: hover,         1: pos_waypoint_timed,      2: pos_waypoint_arrived
    trajSelect[0] = 2       
    # Select Yaw Trajectory Type      (0: none,          1: yaw_waypoint_timed,      2: yaw_waypoint_interp,       3: zero)
    trajSelect[1] = 3           
    # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
    trajSelect[2] = 0           
    print("Control type: {}".format(ctrlType))

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(Ti)
    traj = Trajectory(quad, ctrlType, trajSelect)
    potfld = PotField()
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)

    # Trajectory for First Desired States
    # ---------------------------
    sDes = traj.desiredState(0, Ts, quad) 

    # First Potential Field Calculation
    # ---------------------------
    potfld.isWithinRange(quad)
    potfld.isWithinField(quad)        
    potfld.rep_force(quad)

    # Generate First Commands
    # ---------------------------
    ctrl.controller(traj, quad, sDes, potfld, Ts)
    
    # Initialize Result Matrixes
    # ---------------------------
    numTimeStep = int(Tf/Ts+1)

    t_all          = np.zeros(numTimeStep)
    s_all          = np.zeros([numTimeStep, len(quad.state)])
    pos_all        = np.zeros([numTimeStep, len(quad.pos)])
    vel_all        = np.zeros([numTimeStep, len(quad.vel)])
    quat_all       = np.zeros([numTimeStep, len(quad.quat)])
    omega_all      = np.zeros([numTimeStep, len(quad.omega)])
    euler_all      = np.zeros([numTimeStep, len(quad.euler)])
    sDes_traj_all  = np.zeros([numTimeStep, len(traj.sDes)])
    sDes_calc_all  = np.zeros([numTimeStep, len(ctrl.sDesCalc)])
    w_cmd_all      = np.zeros([numTimeStep, len(ctrl.w_cmd)])
    wMotor_all     = np.zeros([numTimeStep, len(quad.wMotor)])
    thr_all        = np.zeros([numTimeStep, len(quad.thr)])
    tor_all        = np.zeros([numTimeStep, len(quad.tor)])
    notInRange_all = np.zeros([numTimeStep, potfld.num_points], dtype=bool)
    inRange_all    = np.zeros([numTimeStep, potfld.num_points], dtype=bool)
    inField_all    = np.zeros([numTimeStep, potfld.num_points], dtype=bool)
    minDist_all    = np.zeros(numTimeStep)

    t_all[0]            = Ti
    s_all[0,:]          = quad.state
    pos_all[0,:]        = quad.pos
    vel_all[0,:]        = quad.vel
    quat_all[0,:]       = quad.quat
    omega_all[0,:]      = quad.omega
    euler_all[0,:]      = quad.euler
    sDes_traj_all[0,:]  = traj.sDes
    sDes_calc_all[0,:]  = ctrl.sDesCalc
    w_cmd_all[0,:]      = ctrl.w_cmd
    wMotor_all[0,:]     = quad.wMotor
    thr_all[0,:]        = quad.thr
    tor_all[0,:]        = quad.tor
    notInRange_all[0,:] = potfld.notWithinRange
    inRange_all[0,:]    = potfld.inRangeNotField
    inField_all[0,:]    = potfld.withinField
    minDist_all[0]    = potfld.distanceMin

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 1
    while round(t,3) < Tf:
        
        t = quad_sim(t, Ts, quad, ctrl, wind, traj, potfld)
        
        # print("{:.3f}".format(t))
        t_all[i]             = t
        s_all[i,:]           = quad.state
        pos_all[i,:]         = quad.pos
        vel_all[i,:]         = quad.vel
        quat_all[i,:]        = quad.quat
        omega_all[i,:]       = quad.omega
        euler_all[i,:]       = quad.euler
        sDes_traj_all[i,:]   = traj.sDes
        sDes_calc_all[i,:]   = ctrl.sDesCalc
        w_cmd_all[i,:]       = ctrl.w_cmd
        wMotor_all[i,:]      = quad.wMotor
        thr_all[i,:]         = quad.thr
        tor_all[i,:]         = quad.tor
        notInRange_all[i,:]  = potfld.notWithinRange
        inRange_all[i,:]     = potfld.inRangeNotField
        inField_all[i,:]     = potfld.withinField
        minDist_all[i]       = potfld.distanceMin

        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))

    # View Results
    # ---------------------------

    utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all, potfld, minDist_all)
    utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, potfld, notInRange_all, inRange_all, inField_all, ifsave)
    plt.show()

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):
        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))