#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:09:24 2022

@author: 
"""

import numpy as np
import TrajectoryImprovementLibrary as TIL


originalTrajectory = np.load( './SampleTraj/Trj_3.npy' )
targetPoint = -88

variablePoints = np.zeros( ( np.size( originalTrajectory, 0 ) ), dtype = bool )
variablePoints[ targetPoint ] = True
#variablePoints[ targetPoint-1 ] = True
#variablePoints[ targetPoint+1 ] = True

correctedTraj = TIL.TrajectoryImprovement( originalTrajectory, variablePoints, 'random' )

import matplotlib.pyplot as plt
plt.plot( originalTrajectory[ -100:, 1 ], originalTrajectory[ -100:, 2 ], color = 'red' )
plt.plot( correctedTraj[ -100:, 1 ], correctedTraj[ -100:, 2 ], color = 'blue' )
plt.show()
