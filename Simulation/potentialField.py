# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
from numpy import sin, cos, tan, sqrt
from numpy.linalg import norm

rangeRadius = 3
fieldRadius = 2.4


class PotField:

    def __init__(self, pfType):
        self.pointcloud = np.genfromtxt("./Simulation/environmentGeneration/pointcloud_grid.csv", delimiter=",")

        self.num_points = len(self.pointcloud)

        self.max_pc_x = self.pointcloud[:,0].max()
        self.max_pc_y = self.pointcloud[:,1].max()
        self.max_pc_z = self.pointcloud[:,2].max()
        self.min_pc_x = self.pointcloud[:,0].min()
        self.min_pc_y = self.pointcloud[:,1].min()
        self.min_pc_z = self.pointcloud[:,2].min()

        self.center_pc_x = (self.max_pc_x + self.min_pc_x)/2
        self.center_pc_y = (self.max_pc_y + self.min_pc_y)/2
        self.center_pc_z = (self.max_pc_z + self.min_pc_z)/2

        self.withinRange     = np.zeros(self.num_points, dtype=bool)
        self.notWithinRange  = np.zeros(self.num_points, dtype=bool)
        self.withinField     = np.zeros(self.num_points, dtype=bool)
        self.inRangeNotField = np.zeros(self.num_points, dtype=bool)

        self.force = np.zeros(3)
        self.vel   = np.zeros(3)

        self.pfVel = 0
        self.pfSatFor = 0
        self.pfFor = 0
        if (pfType == 1):
            self.pfVel = 1
        elif (pfType == 2):
            self.pfSatFor = 1
        elif (pfType == 3):
            self.pfFor = 0

    
    def isWithinRange(self, quad):
        self.withinRange = (abs(quad.pos[0]-self.pointcloud[:,0]) <= rangeRadius) & \
                           (abs(quad.pos[1]-self.pointcloud[:,1]) <= rangeRadius) & \
                           (abs(quad.pos[2]-self.pointcloud[:,2]) <= rangeRadius)
        self.idx_withinRange = np.where(self.withinRange)[0]
        self.notWithinRange = ~(self.withinRange)
        self.idx_notWithinRange = np.where(self.notWithinRange)[0]

    def isWithinField(self, quad):
        distance = ((self.pointcloud[self.idx_withinRange,0]-quad.pos[0])**2 +          \
                    (self.pointcloud[self.idx_withinRange,1]-quad.pos[1])**2 +          \
                    (self.pointcloud[self.idx_withinRange,2]-quad.pos[2])**2)**(0.5)
        withinField = distance <= fieldRadius
        
        try:
            self.distanceMin = distance.min()
        except ValueError:
            self.distanceMin = -1
        
        self.idx_withinField = self.idx_withinRange[np.where(withinField)[0]]
        self.withinField = np.zeros(self.num_points, dtype=bool)
        self.withinField[self.idx_withinField] = True

        self.idx_inRangeNotField = self.idx_withinRange[np.where(~withinField)[0]]
        self.inRangeNotField = np.zeros(self.num_points, dtype=bool)
        self.inRangeNotField[self.idx_inRangeNotField] = True

        self.fieldPointcloud = self.pointcloud[self.idx_withinField]
        self.fieldDistance = distance[np.where(withinField)[0]]

    def rep_force(self, quad):
        k = 0.32
        F_rep_x = k*(1/self.fieldDistance - 1/fieldRadius)*(1/(self.fieldDistance**2))*(quad.pos[0] - self.fieldPointcloud[:,0])/self.fieldDistance
        F_rep_y = k*(1/self.fieldDistance - 1/fieldRadius)*(1/(self.fieldDistance**2))*(quad.pos[1] - self.fieldPointcloud[:,1])/self.fieldDistance
        F_rep_z = k*(1/self.fieldDistance - 1/fieldRadius)*(1/(self.fieldDistance**2))*(quad.pos[2] - self.fieldPointcloud[:,2])/self.fieldDistance

        self.F_rep = np.array([np.sum(F_rep_x), np.sum(F_rep_y), np.sum(F_rep_z)])        