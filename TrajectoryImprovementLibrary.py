#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:15:21 2022

@author: 
"""

import numpy as np




def TrajectoryImprovement( originalTraj, variablePoints, optAlgorithm ):
    
    
    if optAlgorithm == 'random':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Random( originalTraj, variablePoints, auxParameters )
    
    
    return newTraj





def Algorithm_Random( trajectory, variablePoints, auxParameters ):
    
    Error = aux_CalculateTrajectoryError( trajectory, variablePoints )
    bestTraj = np.copy( trajectory )
    
    for iterN in range( auxParameters[ 'numIters' ] ):
        
        tTraj = np.copy( trajectory )
        for k in range( np.size( trajectory, 0 ) ):
            if variablePoints[ k ]:
                tTraj[ k, 1 ] += np.random.normal( 0.0, 0.01 )
                tTraj[ k, 2 ] += np.random.normal( 0.0, 0.01 )
                
        newError = aux_CalculateTrajectoryError( tTraj, variablePoints )
        
        if newError < Error:
            
            Error = newError
            bestTraj = np.copy( tTraj )
            
            
    return bestTraj




def aux_CalculateTrajectoryError( trajectory, variablePoints ):
    
    totalError = 0.0
    
    for k in range( np.size( variablePoints ) ):
        
        if variablePoints[ k ]:
            totalError += aux_CalculateErrorSinglePoint( trajectory, k )
    
    return totalError




def aux_CalculateErrorSinglePoint( trajectory, targetPoint ):
    
    
    v_m1 = aux_haversine( trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ], \
                          trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ] )
    s_m1 = v_m1 / ( trajectory[ targetPoint, 0 ] - trajectory[ targetPoint - 1, 0 ] )
    
    
    v_m2 = aux_haversine( trajectory[ targetPoint - 2, 2 ], trajectory[ targetPoint - 2, 1 ], \
                          trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ] )
    s_m2 = v_m2 / ( trajectory[ targetPoint - 1, 0 ] - trajectory[ targetPoint - 2, 0 ] )
    
    
    v_p1 = aux_haversine( trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ], \
                          trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ] )
    s_p1 = v_p1 / ( trajectory[ targetPoint + 1, 0 ] - trajectory[ targetPoint, 0 ] )
    
    
    v_p2 = aux_haversine( trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ], \
                          trajectory[ targetPoint + 2, 2 ], trajectory[ targetPoint + 2, 1 ] )
    s_p2 = v_p2 / ( trajectory[ targetPoint + 2, 0 ] - trajectory[ targetPoint + 1, 0 ] )

    return np.std( [ v_m2, v_m1, v_p1, v_p2 ] ) / np.mean( [ v_m2, v_m1, v_p1, v_p2 ] )



def aux_haversine(lon1, lat1, lon2, lat2):
    
    from math import radians, cos, sin, asin, sqrt

    R = 6372.8

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c
