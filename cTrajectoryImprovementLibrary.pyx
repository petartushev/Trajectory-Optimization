#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:15:21 2022

@author: 
"""

from sys import path_importer_cache
import numpy as np
import random
import math
import cython
from libc.stdlib cimport malloc, free

cdef int NUM_ITERS = 1000


def TrajectoryImprovement( originalTraj, variablePoints, optAlgorithm ):
    
    
    if optAlgorithm == 'random':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Random( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'hill_climb':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Hill_Climb( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'simulated_annealing':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Simulated_Annealing( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'tabu':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'local_beam':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters )
    
    
    
    return newTraj





def Algorithm_Random( double[][4] trajectory, bint[] variablePoints, auxParameters ):

    TRAJECTORY_LENGTH = sizeof(trajectory)/sizeof(trajectory[0])

    cdef double[][4] bestTraj = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double)) 
    cdef double *tTraj = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))


    cdef double Error = aux_CalculateTrajectoryError( trajectory, variablePoints )
    bestTraj = trajectory 
    cdef float newError
    
    for iterN in range( auxParameters[ 'numIters' ] ):
        
        tTraj = np.copy( trajectory )

        for k in range( TRAJECTORY_LENGTH ):

            if variablePoints[ k ]:
                tTraj[ k, 1 ] += np.random.normal( 0.0, .001 )
                tTraj[ k, 2 ] += np.random.normal( 0.0, .001 )
                
        newError = aux_CalculateTrajectoryError( tTraj, variablePoints )
        
        if newError < Error:
            
            Error = newError
            bestTraj = np.copy( tTraj )
            
            
    return bestTraj


def Algorithm_Hill_Climb( double[][4] trajectory, bint[] variablePoints, auxParameters ):

    TRAJECTORY_LENGTH = np.array(trajectory).shape[0]

    cdef double *t1Traj = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))
    cdef double *t2Traj = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))
    cdef double *neighbours = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))

    cdef float Error = aux_CalculateTrajectoryError(trajectory, variablePoints)
    bestTraj = np.copy( trajectory )

    cdef double lowest_error
    cdef double error
    cdef double new_path_error

    for inerN in range( auxParameters[ 'numIters' ] ):

        print(f'Iteration: {inerN}')

        t1Traj = np.copy( trajectory )

        for k in range( trajectory.shape[0] ):

            t2Traj = t1Traj 
            
            if variablePoints[k]:

                neighbours = getNeighbours( trajectory[k], auxParameters )

                lowest_error = aux_CalculateTrajectoryError( t1Traj, variablePoints )

                print('Going over newly generated neighbours...')

                for neighbour in neighbours:

                    t2Traj[k] = neighbour

                    error = aux_CalculateTrajectoryError( t2Traj, variablePoints )

                    if error < lowest_error:

                        print('Better path found')

                        t1Traj = t2Traj

                        lowest_error = error
                    
        
        new_path_error = aux_CalculateTrajectoryError( t1Traj, variablePoints )

        if new_path_error < Error:
            bestTraj = np.copy( t1Traj )
            Error = new_path_error

    return bestTraj
                    





def getNeighbours( double[] point, auxParameters ):


    
    cdef double[1000] neighbours
    cdef double[4] neighbour

    for k in range( 1000 ):

        neighbour = np.copy( point )

        neighbour[1] += np.random.normal( 0, .001 )
        neighbour[2] += np.random.normal( 0, .001 )

        neighbours[k] = neighbour

    return neighbours






def Algorithm_Simulated_Annealing( double[][4] originalTraj, bint[] variablePoints, auxParameters ):

    TRAJECTORY_LENGTH = np.array(trajectory).shape[0]

    cdef double *bestTraj = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))
    cdef double *solution = <double *> malloc(TRAJECTORY_LENGTH*4*sizeof(double))

    cdef float initial_temp = 90
    cdef float final_temp = .1
    cdef float alpha = .01

    cdef float current_temp = initial_temp
    cdef double neighbour

    bestTraj = np.copy( originalTraj )

    

    while current_temp > final_temp:


        for k in range( originalTraj.shape[0] ):

            solution = bestTraj
            
            if variablePoints[k]:

                neighbour = random.choice(getNeighbours(solution[k], auxParameters))

                solution[k + 1] = neighbour

                cost_diff = aux_CalculateErrorSinglePoint(bestTraj, k + 1) - aux_CalculateErrorSinglePoint(solution, k + 1)

                if cost_diff > 0:

                    print('New path found.')

                    bestTraj = np.copy( solution )

                else:

                    if random.uniform(0, 1) < math.exp(-cost_diff/current_temp):

                        print('New path found.')

                        bestTraj = np.copy( solution )

        current_temp -= alpha

    return bestTraj


def Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters ):

    bestTraj = np.copy( originalTraj )
    Error = aux_CalculateTrajectoryError( bestTraj, variablePoints )

    # aspirationCriteria = 2
    tabuTenure = 20
    tabuList = dict()

    for k in range( len( originalTraj ) ):

        currTraj = np.copy( bestTraj )

        if variablePoints[k]:

            neighbours = getNeighbours( currTraj[k], auxParameters )

            # neighbours = filter(lambda x: aspirationCriteria(x))

            bestNeighbour = None

            for neighbour in neighbours:

                currTraj[k + 1] = neighbour

                error = aux_CalculateTrajectoryError( currTraj, variablePoints )

                cost_diff = Error - error

                if cost_diff <= 0:

                    bestTraj = np.copy( currTraj )

                    bestNeighbour = neighbour

        for key in tabuList.keys():
            
            tabuList[key] -= 1
            
            if tabuList[key] == 0:
                del tabuList[key]

        tabuList[bestNeighbour] = tabuTenure

    return bestTraj






def aspirationCriteria(point):
    raise NotImplementedError()



def Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters ):
    raise NotImplementedError


def Genetic_Algorithm( originalTraj, variablePoints, auxParameters ):
    # prvo ke gi najdam site True variable variablePoints
    # za sekoj variablePoint, vklucuvajki go i veke postoeckio, 
    # ke im napravam selekcija so nekojasi zasega nespecificirana fitness funkcija
    # ke napravam vkrstuvanje i na odreden procent ke napravam mutacija
    # i taka tava ke go praam 1000 pati
    # posle tava, za sekoj variable point ke ga izberam najdobrata tocka
    # ke ga vratam traektorijata

    raise NotImplementedError()




cdef aux_CalculateTrajectoryError( double[][4] trajectory, bint[] variablePoints ):
    
    cdef double totalError = 0.0
    
    for k in range( np.size( variablePoints ) ):
        
        if variablePoints[ k ]:
            totalError += aux_CalculateErrorSinglePoint( trajectory, k )
    
    return totalError




def aux_CalculateErrorSinglePoint( double[][4] trajectory, int targetPoint ):
    
    
    cdef double v_m1 = c_aux_haversine( trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ], trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ] )
    s_m1 = v_m1 / ( trajectory[ targetPoint, 0 ] - trajectory[ targetPoint - 1, 0 ] )
    
    
    cdef double v_m2 = c_aux_haversine( trajectory[ targetPoint - 2, 2 ], trajectory[ targetPoint - 2, 1 ], trajectory[ targetPoint - 1, 2 ], trajectory[ targetPoint - 1, 1 ] )
    s_m2 = v_m2 / ( trajectory[ targetPoint - 1, 0 ] - trajectory[ targetPoint - 2, 0 ] )
    
    
    cdef double v_p1 = c_aux_haversine( trajectory[ targetPoint, 2 ], trajectory[ targetPoint, 1 ], trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ] )
    s_p1 = v_p1 / ( trajectory[ targetPoint + 1, 0 ] - trajectory[ targetPoint, 0 ] )
    
    
    cdef double v_p2 = c_aux_haversine( trajectory[ targetPoint + 1, 2 ], trajectory[ targetPoint + 1, 1 ], trajectory[ targetPoint + 2, 2 ], trajectory[ targetPoint + 2, 1 ] )
    s_p2 = v_p2 / ( trajectory[ targetPoint + 2, 0 ] - trajectory[ targetPoint + 1, 0 ] )

    return np.std( [ v_m2, v_m1, v_p1, v_p2 ] ) / np.mean( [ v_m2, v_m1, v_p1, v_p2 ] )



def aux_haversine(double lon1, double lat1, double lon2, double lat2): # haversine is the angle distance between two points on a sphere 
    
    from math import radians, cos, sin, asin, sqrt

    cdef double R = 6372.8

    cdef double dLat = radians(lat2 - lat1)
    cdef double dLon = radians(lon2 - lon1)
    lat1 = radians(lat1)
    lat2 = radians(lat2)

    cdef double a = sin(dLat/2)**2 + cos(lat1)*cos(lat2)*sin(dLon/2)**2
    cdef double c = 2*asin(sqrt(a))

    return R * c
