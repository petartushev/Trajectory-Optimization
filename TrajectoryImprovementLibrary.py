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

    if optAlgorithm == 'hill_climb':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Hill_Climb( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'simulated_annealing':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Simulated_Annealing( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'tabu':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'local_beam':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters )
    
    
    
    return newTraj





def Algorithm_Random( trajectory, variablePoints, auxParameters ):
    
    Error = aux_CalculateTrajectoryError( trajectory, variablePoints )
    bestTraj = np.copy( trajectory )
    
    for iterN in range( auxParameters[ 'numIters' ] ):
        
        tTraj = np.copy( trajectory )
        for k in range( np.size( trajectory, 0 ) ):
            if variablePoints[ k ]:
                tTraj[ k, 1 ] += np.random.normal( 0.0, 5 )
                tTraj[ k, 2 ] += np.random.normal( 0.0, 5 )
                
        newError = aux_CalculateTrajectoryError( tTraj, variablePoints )
        
        if newError < Error:
            
            Error = newError
            bestTraj = np.copy( tTraj )
            
            
    return bestTraj


def Algorithm_Hill_Climb( trajectory, variablePoints, auxParameters ):

    Error = aux_CalculateTrajectoryError(trajectory, variablePoints)
    bestTraj = np.copy( trajectory )


    for inerN in range( auxParameters[ 'numIters' ] ):

        t1Traj = np.copy( trajectory )

        for k in range( trajectory.shape[0] ):

            t2Traj = np.copy(t1Traj)
            
            neighbours = []
            
            if variablePoints[k]:

                neighbours = getNeighbours( trajectory[k], auxParameters )

                # bestNeighbour = t2Traj[k]

                lowest_error = aux_CalculateTrajectoryError( t1Traj, variablePoints )

                for neighbour in neighbours:

                    t2Traj[k] = neighbour

                    error = aux_CalculateTrajectoryError( t2Traj, variablePoints )

                    if error < lowest_error:

                        t1Traj = t2Traj

                        lowest_error = error
                    
        
        new_path_error = aux_CalculateTrajectoryError( t1Traj, variablePoints )

        if new_path_error < Error:
            bestTraj = np.copy( t1Traj )
            Error = new_path_error

    return bestTraj
                    





def getNeighbours( point, auxParameters ):
    neighbours = []

    for _ in auxParameters[ 'numIters' ]:

        neighbour = np.copy( point )

        neighbour[1] += np.random.normal( 0, .1 )
        neighbour[2] += np.random.normal( 0, .1 )

        neighbours.append(neighbour)

    return neighbours






def Algorithm_Simulated_Annealing( originalTraj, variablePoints, auxParameters ):

    initial_temp = 90
    final_temp = .1
    alpha = .01

    current_temp = initial_temp

    bestTraj = np.copy( originalTraj )
    

    while current_temp > final_temp:

        for k in range( len( originalTraj.shape[0] ) ):

            solution = np.copy( bestTraj )
            
            if variablePoints[k]:

                neighbour = random.choice(getNeighbours(solution[k], auxParameters))

                solution[k + 1] = neighbour

                cost_diff = aux_CalculateErrorSinglePoint(bestTraj, k + 1) - aux_CalculateErrorSinglePoint(solution, k + 1)

                if cost_diff > 0:

                    bestTraj = np.copy( solution )

                else:

                    if random.uniform(0, 1) < math.exp(-cost_diff/current_temp):

                        bestTraj = np.copy( solution )

        current_temp -= alpha

    return bestTraj


def Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters ):
    raise NotImplementedError

def Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters ):
    raise NotImplementedError




def aux_CalculateTrajectoryError( trajectory, variablePoints ):
    
    totalError = 0.0
    
    for k in range( np.size( variablePoints ) ):
        
        if variablePoints[ k ]:
            totalError += aux_CalculateErrorSinglePoint( trajectory, k )
    
    return totalError




def aux_CalculateErrorSinglePoint( trajectory, targetPoint ):
    
    
    v_m1 = aux_haversine( trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ], trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ] )
    s_m1 = v_m1 / ( trajectory[ targetPoint, 0 ] - trajectory[ targetPoint - 1, 0 ] )
    
    
    v_m2 = aux_haversine( trajectory[ targetPoint - 2, 2 ], trajectory[ targetPoint - 2, 1 ], trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ] )
    s_m2 = v_m2 / ( trajectory[ targetPoint - 1, 0 ] - trajectory[ targetPoint - 2, 0 ] )
    
    
    v_p1 = aux_haversine( trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ], trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ] )
    s_p1 = v_p1 / ( trajectory[ targetPoint + 1, 0 ] - trajectory[ targetPoint, 0 ] )
    
    
    v_p2 = aux_haversine( trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ], trajectory[ targetPoint + 2, 2 ], trajectory[ targetPoint + 2, 1 ] )
    s_p2 = v_p2 / ( trajectory[ targetPoint + 2, 0 ] - trajectory[ targetPoint + 1, 0 ] )

    return np.std( [ v_m2, v_m1, v_p1, v_p2 ] ) / np.mean( [ v_m2, v_m1, v_p1, v_p2 ] )



def aux_haversine(lon1, lat1, lon2, lat2): # haversine is the angle distance between two points on a sphere 
    
    from math import radians, cos, sin, asin, sqrt

    R = 6372.8

    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))

    return R * c
