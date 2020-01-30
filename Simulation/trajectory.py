# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# Functions get_poly_cc, minSomethingTraj, pos_waypoint_min are derived from Peter Huang's work:
# https://github.com/hbd730/quadcopter-simulation
# author: Peter Huang
# email: hbd730@gmail.com
# license: BSD
# Please feel free to use and modify this, but keep the above information. Thanks!



import numpy as np
from numpy import pi
from numpy.linalg import norm
from waypoints import makeWaypoints
import config

class Trajectory:

    def __init__(self, quad, ctrlType, trajSelect):

        self.ctrlType = ctrlType
        self.xyzType = trajSelect[0]
        self.yawType = trajSelect[1]
        self.averVel = trajSelect[2]

        t_wps, wps, y_wps, v_wp = makeWaypoints()
        self.t_wps = t_wps
        self.wps   = wps
        self.y_wps = y_wps
        self.v_wp  = v_wp

        self.end_reached = 0

        if (self.ctrlType == "xyz_pos"):
            self.T_segment = np.diff(self.t_wps)

            if (self.averVel == 1):
                distance_segment = self.wps[1:] - self.wps[:-1]
                self.T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/self.v_wp
                self.t_wps = np.zeros(len(self.T_segment) + 1)
                self.t_wps[1:] = np.cumsum(self.T_segment)
        
        if (self.yawType == 4):
            self.y_wps = np.zeros(len(self.t_wps))
        
        # Get initial heading
        self.current_heading = quad.psi
        
        # Initialize trajectory setpoint
        self.desPos = np.zeros(3)    # Desired position (x, y, z)
        self.desVel = np.zeros(3)    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.zeros(3)    # Desired thrust in N-E-D directions (or E-N-U, if selected)
        self.desEul = np.zeros(3)    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.zeros(3)    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.         # Desired yaw speed
        self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate)).astype(float)


    def desiredState(self, t, Ts, quad):
        
        self.desPos = np.zeros(3)    # Desired position (x, y, z)
        self.desVel = np.zeros(3)    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.zeros(3)    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.zeros(3)    # Desired thrust in N-E-D directions (or E-N-U, if selected)
        self.desEul = np.zeros(3)    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.zeros(3)    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.         # Desired yaw speed


        def pos_waypoint_timed():
            
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
            elif (np.diff(self.t_wps) <= 0).any():
                raise Exception("Time array isn't properly ordered.")  
            
            if (t == 0):
                self.t_idx = 0
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
            
            self.desPos = self.wps[self.t_idx,:]
                            
        
        def pos_waypoint_arrived():

            dist_consider_arrived = 0.2 # Distance to waypoint that is considered as "arrived"
            if (t == 0):
                self.t_idx = 0
                self.end_reached = 0
            elif not(self.end_reached):
                distance_to_next_wp = ((self.wps[self.t_idx,0]-quad.pos[0])**2 + (self.wps[self.t_idx,1]-quad.pos[1])**2 + (self.wps[self.t_idx,2]-quad.pos[2])**2)**(0.5)
                if (distance_to_next_wp < dist_consider_arrived):
                    self.t_idx += 1
                    if (self.t_idx >= len(self.wps[:,0])):    # if t_idx has reached the end of planned waypoints
                        self.end_reached = 1
                        self.t_idx = -1
                    
            self.desPos = self.wps[self.t_idx,:]


        def yaw_waypoint_timed():
            
            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")
            
            self.desEul[2] = self.y_wps[self.t_idx]
                    

        def yaw_waypoint_interp():

            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")

            if (t == 0) or (t >= self.t_wps[-1]):
                self.desEul[2] = self.y_wps[self.t_idx]
            else:
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desEul[2] = (1 - scale)*self.y_wps[self.t_idx] + scale*self.y_wps[self.t_idx + 1]
                
                # Angle between current vector with the next heading vector
                delta_psi = self.desEul[2] - self.current_heading
                
                # Set Yaw rate
                self.desYawRate = delta_psi / Ts 

                # Prepare next iteration
                self.current_heading = self.desEul[2]


        if (self.ctrlType == "xyz_vel"):
            if (self.xyzType == 1):
                self.sDes = testVelControl(t)

        elif (self.ctrlType == "xy_vel_z_pos"):
            if (self.xyzType == 1):
                self.sDes = testVelControl(t)
        
        elif (self.ctrlType == "xyz_pos"):
            # Hover at [0, 0, 0]
            if (self.xyzType == 0):
                pass 
            # For simple testing
            elif (self.xyzType == 99):
                self.sDes = testXYZposition(t)   
            else:    
                # List of possible position trajectories
                # ---------------------------
                # Set desired positions at every t_wps[i]
                if (self.xyzType == 1):
                    pos_waypoint_timed()
                # Go to next waypoint when arrived at waypoint
                elif (self.xyzType == 2):
                    pos_waypoint_arrived()
                
                # List of possible yaw trajectories
                # ---------------------------
                # Set desired yaw at every t_wps[i]
                if (self.yawType == 0):
                    pass
                elif (self.yawType == 1):
                    yaw_waypoint_timed()
                # Interpolate yaw between every waypoint, to arrive at desired yaw every t_wps[i]
                elif (self.yawType == 2):
                    yaw_waypoint_interp()

                self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate)).astype(float)
        
        return self.sDes






## Testing scripts

def testXYZposition(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 30.0*pi/180
    
    if t >= 1 and t < 4:
        desPos = np.array([2, 2, 1])
    elif t >= 4:
        desPos = np.array([2, -2, -2])
        desEul = np.array([0, 0, pi/3])
    
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)

    return sDes


def testVelControl(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 0.

    if t >= 1 and t < 4:
        desVel = np.array([3, 2, 0])
    elif t >= 4:
        desVel = np.array([3, -1, 0])
     
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)
    
    return sDes
