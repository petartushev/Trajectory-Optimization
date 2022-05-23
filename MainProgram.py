#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:09:24 2022

@author: 
"""
import os
import numpy as np
import TrajectoryImprovementLibrary as TIL

trajectory1 = np.load( '/home/petar/Fakultet/Semester 8/Biologically inspired computing/Proekt/SampleTraj/Trj_0.npy' )
trajectory2 = np.load( '/home/petar/Fakultet/Semester 8/Biologically inspired computing/Proekt/SampleTraj/Trj_1.npy' )
trajectory3 = np.load( '/home/petar/Fakultet/Semester 8/Biologically inspired computing/Proekt/SampleTraj/Trj_2.npy' )
trajectory4 = np.load( '/home/petar/Fakultet/Semester 8/Biologically inspired computing/Proekt/SampleTraj/Trj_3.npy' )

# originalTrajectory = np.load( './SampleTraj/Trj_3.npy' )
targetPoint = -88

variablePoints = np.zeros( ( np.size( trajectory3, 0 ) ), dtype = bool )
variablePoints[ targetPoint ] = True
#variablePoints[ targetPoint-1 ] = True
#variablePoints[ targetPoint+1 ] = True

correctedTraj = TIL.TrajectoryImprovement( trajectory3, variablePoints, 'genetic' )

# import matplotlib.pyplot as plt
# plt.plot( originalTrajectory[ -100:, 1 ], originalTrajectory[ -100:, 2 ], color = 'red' )
# plt.plot( correctedTraj[ -100:, 1 ], correctedTraj[ -100:, 2 ], color = 'blue' )
# plt.show()
