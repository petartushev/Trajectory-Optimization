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
from itertools import tee




def TrajectoryImprovement( originalTraj, variablePoints, optAlgorithm ):
    
    
    if optAlgorithm == 'random':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Random( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'hill_climb':
        auxParameters = { 'numIters': 10 }
        newTraj = Algorithm_Hill_Climb( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'simulated_annealing':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Simulated_Annealing( originalTraj, variablePoints )

    if optAlgorithm == 'tabu':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'local_beam':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters )


    if optAlgorithm == 'genetic':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters )
    
    
    
    return newTraj





def Algorithm_Random( trajectory, variablePoints, auxParameters ):
    
    Error = aux_CalculateTrajectoryError( trajectory, variablePoints )
    bestTraj = np.copy( trajectory )
    
    for iterN in range( auxParameters[ 'numIters' ] ):
        
        tTraj = np.copy( trajectory )
        for k in range( np.size( trajectory, 0 ) ):
            if variablePoints[ k ]:
                tTraj[ k, 1 ] += np.random.normal( 0.0, .01 )
                tTraj[ k, 2 ] += np.random.normal( 0.0, .01 )
                
        newError = aux_CalculateTrajectoryError( tTraj, variablePoints )
        
        if newError < Error:
            
            Error = newError
            bestTraj = np.copy( tTraj )
            
            
    return bestTraj


def Algorithm_Hill_Climb( trajectory, variablePoints, auxParameters ):

    Error = aux_CalculateTrajectoryError(trajectory, variablePoints)
    bestTraj = np.copy( trajectory )


    for inerN in range( auxParameters[ 'numIters' ] ):

        print(f'Iteration: {inerN}')

        t1Traj = np.copy( trajectory )

        for k in range( trajectory.shape[0] ):

            t2Traj = np.copy(t1Traj)
            
            neighbours = []
            
            if variablePoints[k]:

                neighbours = getNeighbours( trajectory[k] )


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
                    





def Algorithm_Simulated_Annealing( originalTraj, variablePoints ):

    initial_temp = 90
    final_temp = .1
    alpha = .1

    current_temp = initial_temp

    bestTraj = np.copy( originalTraj )
    

    while current_temp > final_temp:
        print(f'Current temp: {current_temp}')


        for k in range( originalTraj.shape[0] ):

            solution = np.copy( bestTraj )
            
            if variablePoints[k]:

                neighbour = random.choice(getNeighbours( solution[k] ))

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
        
        bestNeighbour = None

        currTraj = np.copy( bestTraj )

        if variablePoints[k]:

            neighbours = getNeighbours( currTraj[k] )

            # neighbours = filter(lambda x: aspirationCriteria(x))

            

            for neighbour in neighbours:
                # print(f'Neighbour: {neighbour}')  

                currTraj[k + 1] = neighbour

                error = aux_CalculateTrajectoryError( currTraj, variablePoints )

                cost_diff = Error - error

                if cost_diff >= 0:
                    # print('New path found.')
                    bestTraj = np.copy( currTraj )

                    bestNeighbour = neighbour


        for key in tabuList.copy().keys():
            
            tabuList[key] -= 1
            
            if tabuList[key] == 0:
                del tabuList[key]
                # to_be_removed.append(key)

        if bestNeighbour is not None:
            # print(bestNeighbour)
            tabuList[bestNeighbour.tobytes()] = tabuTenure

        # for key in to_be_removed:
        #     del tabuList[key]

    return bestTraj






# def aspirationCriteria(point):
#     raise NotImplementedError()



def Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters ):
    print()


def Genetic_Algorithm( originalTraj, variablePoints, auxParameters ):
    # 1. Inicijalna populacija: treba za variablePoints deka so se True, da izgeneriram odreden broj na edinki
    # 2. So fitnes funkcijata, u taj slucaj aux_CalculateErrorSinglePoint, za sekoja ednika da presmetam kolku e dobra
    # 3. Ko ke presmetam fitness, treba da go zacuvam toa vo dict i da gi sortiram spored vrednosta. Prvata polovina ke bide odbrana za ponatamu.
    # 4. treba pomegju edinkite nekako da napravam crossover t.e. nekako da gi vkrstam za da dobijam duplo
    # 5. Treba da napravam so odreden procent od novite edinki da se imat mutirano


    bestTraj = np.copy( originalTraj )

    best_units = dict()

    for iter in range(auxParameters[ 'numIters' ]):

        # print('STEP 1')

        d = dict()

        for k in range(len( originalTraj )):                            # STEP 1

            if variablePoints[k]:

                d[k] = getNeighbours(originalTraj[k])


        index_to_errors = dict()


        # print('STEP 2')

        for key in d.keys():                                            # STEP 2

            tTraj = np.copy( bestTraj )

            individual_errors = dict()

            for individual in d[key]:

                tTraj[key] = individual

                error = aux_CalculateTrajectoryError(tTraj, variablePoints)

                individual_errors[individual.tobytes()] = error

                if key in best_units.keys():

                    tTraj[key] = best_units[key]

                    best_error = aux_CalculateTrajectoryError(tTraj, variablePoints)

                    if best_error > error:

                        best_units[key] = individual

                else:

                    best_units[key] = individual

            index_to_errors[key] = individual_errors.copy()

        index_to_errors_sorted = dict()
        selected_individuals = dict()


        # print('STEP 3')

        for key in index_to_errors.keys():                              # STEP 3

            index_to_errors_sorted[key] = sorted(index_to_errors[key].items(), key=lambda x: x[1], reverse=False)

            selected_individuals[key] = dict(list(index_to_errors_sorted[key])[:int(len(index_to_errors_sorted[key])/2)])


        # print('STEP 4')

        for key in selected_individuals.keys():                         # STEP 4
            
            #treba da dojdam do edinkite, da gi vratam vo float64 ndarray format od bytes, error ne mi treba veke
            # treba da gi zemam dva po dva elementite, i da gi zamenam mestata na lon koordinatite

            keys = selected_individuals[key].keys()

            keys = np.array([np.frombuffer(x) for x in keys])

            for k in range(0, len(keys), 2):
                            
                keys[k][2], keys[k + 1][2] = keys[k + 1][2], keys[k][2]

                # print('STEP 5')

                if random.random() < .2:                                # STEP 5

                    keys[k][1] += np.random.normal(0, .001)
                    keys[k][2] += np.random.normal(0, .001)

    for key, value in best_units.items():

        bestTraj[key] = value

    return bestTraj
      

               


def getNeighbours( point ):
    neighbours = []

    for _ in range( 500 ):

        neighbour = np.copy( point )

        neighbour[1] += np.random.normal( 0, .01 )
        neighbour[2] += np.random.normal( 0, .01 )

        neighbours.append(neighbour)

    return neighbours




def pairwise(iterable):

    a, b = tee(iterable)
    next(b, None)

    return zip(a, b)
                



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
