#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 20 15:15:21 2022

@author: 
"""

import os
import sys

import numpy as np
import random
import math

from copy import deepcopy
from six import add_metaclass
from abc import ABCMeta
from IPython.display import display
import folium

# print(sys.path)
# print(os.path.realpath(__file__))
from loss_functions import aux_CalculateTrajectoryError


def TrajectoryImprovement( originalTraj, variablePoints, optAlgorithm ):
    
    
    if optAlgorithm == 'random':
        auxParameters = { 'numIters': 1000 }
        newTraj = Algorithm_Random( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'hill_climb':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Hill_Climb( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'simulated_annealing':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Simulated_Annealing( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'tabu':
        auxParameters = { 'numIters': 1 }
        newTraj, markers = Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'local_beam':
        auxParameters = { 'numIters': 1 }
        newTraj = Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'genetic':
        auxParameters = { 'numIters': 1 }
        newTraj = Genetic_Algorithm( originalTraj, variablePoints, auxParameters )

    if optAlgorithm == 'pso':
        auxParameters = { 'numIters': 13 }
        newTraj = Algorithm_Particle_Swarm(originalTraj, variablePoints, auxParameters)

    
    
    
    return newTraj

def map_error_points(traj, k):

    m = folium.Map(
    location=[traj[0][1], traj[0][2]],
    tiles="cartodbpositron",
    zoom_start=5.7,
    )

    aline1 = folium.PolyLine(traj[:, 1:3], color='blue')
    m.add_child(aline1)

    folium.Marker(location=[traj[k-2][1], traj[k-2][2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)
    folium.Marker(location=[traj[k-1][1], traj[k-1][2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)
    folium.Marker(location=[traj[k][1], traj[k][2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)
    folium.Marker(location=[traj[k+1][1], traj[k+1][2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)
    folium.Marker(location=[traj[k+2][1], traj[k+2][2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)

    display(m)
    # m.save(f'maps/map{temp_counter}.html')
    # temp_counter += 1
    # webbrowser.open('map1.html')


def map_generated_points(trajectory, k, generated_points):

    m = folium.Map(
    location=[trajectory[k][1], trajectory[k][2]],
    tiles="cartodbpositron",
    zoom_start=5.7,
    )

    aline1 = folium.PolyLine(trajectory[:, 1:3], color='blue')
    m.add_child(aline1)

    for point in generated_points:
        folium.Marker(location=[point[1], point[2]], icon=folium.Icon(color='darkblue', icon_color='white', icon='male', angle=0, prefix='fa')).add_to(m)

    display(m)










def Algorithm_Random( trajectory, variablePoints, auxParameters ):
    
    Error = aux_CalculateTrajectoryError( trajectory, variablePoints )
    bestTraj = np.copy( trajectory )
    
    for iterN in range( auxParameters[ 'numIters' ] ):
        
        tTraj = np.copy( trajectory )
        for k in range( np.size( trajectory, 0 ) ):
            if variablePoints[ k ]:
                tTraj[ k, 1 ] += np.random.normal( 0.0, .001 )
                tTraj[ k, 2 ] += np.random.normal( 0.0, .001 )
                
        newError = aux_CalculateTrajectoryError( tTraj, variablePoints )
        
        if newError < Error:
            
            Error = newError
            bestTraj = np.copy( tTraj )
            
            
    return bestTraj


def Algorithm_Hill_Climb( trajectory, variablePoints, auxParameters ):

    bestTraj = np.copy( trajectory )
    STD_DEV = .0001

    for _ in range( auxParameters[ 'numIters' ] ):

        t1Traj = np.copy( bestTraj )

        for k in np.where(variablePoints)[0]:

            t2Traj = np.copy(t1Traj)

            neighbours = getNeighbours( t1Traj[k], n=2000, STD_DEV=STD_DEV )

            lowest_error = aux_CalculateTrajectoryError( t1Traj, variablePoints )

            for neighbour in neighbours:

                t2Traj[k + 1] = np.copy( neighbour )

                error = aux_CalculateTrajectoryError( t2Traj, variablePoints )

                if error < lowest_error:

                    t1Traj[k + 1] = np.copy( neighbour )

                    lowest_error = error

        bestTraj = np.copy(t1Traj)

    return bestTraj
                    





def Algorithm_Simulated_Annealing( originalTraj, variablePoints, auxParameters ):

    initial_temp = 90
    final_temp = .1
    alpha = 1
    STD_DEV = .001

    current_temp = initial_temp

    bestTraj = np.copy( originalTraj )

    for _ in range( auxParameters['numIters']):
    

        while current_temp > final_temp:
            # print(f'Current temp: {current_temp}')


            for k in np.where(variablePoints)[0]:

                solution = np.copy( bestTraj )
                
                # if variablePoints[k]:

                # neighbours = 

                # print(f'Getting neighbours for: {k}')

                for neighbour in getNeighbours( solution[k], n=100, STD_DEV=STD_DEV ):

                # neighbour = random.choice(  )

                    solution[k + 1] = np.copy( neighbour )

                    cost_diff = aux_CalculateTrajectoryError(bestTraj, variablePoints) - aux_CalculateTrajectoryError(solution, variablePoints)

                    if cost_diff > 0:

                        # print('New path found.')

                        bestTraj = np.copy( solution )

                    else:

                        if random.uniform(0, 1) < math.exp(-cost_diff/current_temp):

                            # print('New path found.')

                            bestTraj = np.copy( solution )

            current_temp -= alpha

        current_temp = initial_temp

    return bestTraj


def Algorithm_Tabu_Search( originalTraj, variablePoints, auxParameters ):

    bestTraj = np.copy( originalTraj )
    Error = aux_CalculateTrajectoryError( bestTraj, variablePoints )
    STD_DEV = .00005

    # aspirationCriteria = 2
    tabuTenure = 20
    
    markers = dict()
    
    for _ in range( auxParameters['numIters'] ):

        tabuList = dict()

        for k in np.where(variablePoints)[0]:
            
            bestNeighbour = None

            currTraj = np.copy( bestTraj )

            # print(f'K in function: {k}')

            neighbours = getNeighbours( currTraj[k], n=1000, STD_DEV=STD_DEV)

            # markers[k] = np.copy(neighbours)

            # neighbours = filter(lambda x: aspirationCriteria(x))

            for neighbour in neighbours:

                currTraj[k + 1] = np.copy(neighbour)

                error = aux_CalculateTrajectoryError( currTraj, variablePoints )

                # print(f'Old loss: {Error}')
                # print(f'New loss: {error}')

                cost_diff = Error - error

                # print(f'Cost diff: {cost_diff}')

                if cost_diff >= 0:

                    bestTraj = np.copy( currTraj )
                    Error = error

                    bestNeighbour = neighbour

                    # print('New path found.')


            for key in tabuList.copy().keys():
                
                tabuList[key] -= 1
                
                if tabuList[key] == 0:
                    del tabuList[key]

            if bestNeighbour is not None:
                tabuList[bestNeighbour.tobytes()] = tabuTenure

    return bestTraj, markers



def Algorithm_Local_Beam( originalTraj, variablePoints, auxParameters, beta=3 ):
    
    bestTraj = originalTraj
    Error = aux_CalculateTrajectoryError(originalTraj, variablePoints)
    STD_DEV = .00005

    # print(Error)

    for _ in range( auxParameters['numIters'] ):

        tier_to_paths = dict()

        for k in np.where(variablePoints)[0]:

            tmp_path = np.copy(bestTraj)

            # if variablePoints[k]:

            # vo tier_to_paths dictionary da inicijaliziram na tocka k u traektorijata prazna numpy.array

            tier_to_paths[k] = []

            # za sekoj variable point da izgeneriram novi points

            neighbours = getNeighbours( tmp_path[k], n=1000, STD_DEV=STD_DEV )

            # za sekoj izgeneriran nov point da presmetam haversine distance

            for neighbour in neighbours:

                tmp_path[k + 1] = neighbour

                error = aux_CalculateTrajectoryError(tmp_path, variablePoints)

                if error < Error:

                    tier_to_paths[k].append(np.array([neighbour, error]))

            # najdobrite beta za k variable point da gi zacuvam 
            tier_to_paths[k] = sorted(tier_to_paths[k], key=lambda x: x[1], reverse=False)[:beta]
            
            # print(tier_to_paths[k])
            # print()

            if len(tier_to_paths[k]) > 0:
                if tier_to_paths[k][0][1] < Error:
                    
                    # print(tier_to_paths[k][0])

                    tmp_path[k + 1] = tier_to_paths[k][0][0]

                    bestTraj = np.copy(tmp_path)
                    Error = tier_to_paths[k][0][1]

    return bestTraj







def Genetic_Algorithm( originalTraj, variablePoints, auxParameters ):
    # 1. Inicijalna populacija: treba za variablePoints deka so se True, da izgeneriram odreden broj na edinki
    # 2. So fitnes funkcijata, u taj slucaj aux_CalculateErrorSinglePoint, za sekoja ednika da presmetam kolku e dobra
    # 3. Ko ke presmetam fitness, treba da go zacuvam toa vo dict i da gi sortiram spored vrednosta. Prvata polovina ke bide odbrana za ponatamu.
    # 4. treba pomegju edinkite nekako da napravam crossover t.e. nekako da gi vkrstam za da dobijam duplo
    # 5. Treba da napravam so odreden procent od novite edinki da se imat mutirano


    bestTraj = np.copy( originalTraj )
    Error = aux_CalculateTrajectoryError(originalTraj, variablePoints)

    selected_individuals = dict()
    flag = True
    n = 1000
    STD_DEV = .00005

    for _ in range(auxParameters[ 'numIters' ]):

        # print('STEP 1')

        d = dict()

        for k in np.where(variablePoints)[0]:                            # STEP 1

            if flag:

                selected_individuals[k] = dict()            

            d[k] = getNeighbours( bestTraj[k], n )
 
        flag=False

        index_to_errors = dict()


        # print('STEP 2')

        for key in d.keys():                                            # STEP 2

            tTraj = np.copy( bestTraj )

            individual_errors = dict()

            for individual in d[key]:

                tTraj[key + 1] = np.copy(individual)

                error = aux_CalculateTrajectoryError(tTraj, variablePoints)

                individual_errors[np.copy(individual).tobytes()] = error

            index_to_errors[key] = individual_errors.copy()
    
        index_to_errors_sorted = dict()

        # print('STEP 3')

        for key in index_to_errors.keys():                              # STEP 3

            index_to_errors_sorted[key] = dict(sorted(index_to_errors[key].items(), key=lambda x: x[1], reverse=False))

            selected_individuals[key].update(dict(list(index_to_errors_sorted[key].items())[:int(len(index_to_errors_sorted[key])*0.25)]))


        # print('STEP 4')

        for key in selected_individuals.keys():                         # STEP 4
            
            #treba da dojdam do edinkite, da gi vratam vo float64 ndarray format od bytes, error ne mi treba veke
            # treba da gi zemam dva po dva elementite, i da gi zamenam mestata na lon koordinatite

            keys = selected_individuals[key].keys()

            keys = np.array([np.frombuffer(x) for x in keys])

            for k in range(0, len(keys), 2):

                if (k + 1) == len(keys):
                    break
                
                k1, k2 = np.copy(keys[k]), np.copy(keys[k + 1])
       
                k1[2], k2[2] = k2[2], k1[2]
                k1[1], k2[1] = k2[1], k1[1]

                # print('STEP 5')

                if random.random() < .2:                                # STEP 5

                    k1[1] += np.random.normal(0, STD_DEV)
                    k1[2] += np.random.normal(0, STD_DEV)

                    k2[1] += np.random.normal(0, STD_DEV)
                    k2[2] += np.random.normal(0, STD_DEV)

                tTraj = np.copy( bestTraj )
                
                tTraj[key] = k1.copy()
                t1 = np.copy(tTraj)

                tTraj = np.copy( bestTraj )

                tTraj[key+1] = k2
                t2 = np.copy(tTraj)

                error1 = aux_CalculateTrajectoryError(t1, variablePoints)
                error2 = aux_CalculateTrajectoryError(t2, variablePoints)
                
                if selected_individuals[key][keys[k].tobytes()] <= error1:
                    del selected_individuals[key][keys[k].tobytes()]

                if selected_individuals[key][keys[k + 1].tobytes()] <= error1:
                    del selected_individuals[key][keys[k + 1].tobytes()]

                selected_individuals[key][k1.tobytes()] = error1
                selected_individuals[key][k2.tobytes()] = error2

            selected_individuals[key] = dict(sorted(selected_individuals[key].items(), key=lambda x: x[1], reverse=False))
                
            error = list(selected_individuals[key].values())[0]

            if Error > error:

                bestTraj[key + 1] = np.frombuffer(list(selected_individuals[key].keys())[0])
                Error = error

    return bestTraj


def Algorithm_Particle_Swarm(trajectory, variablePoints, auxParameters):

    pso = PSO(trajectory, variablePoints, auxParameters['numIters'])

    return pso.run()


# Treba hiperparametrite na velocity da se tunirat

class Particle:

    VELOCITY = .1
    STD_DEV = .0001

    def __init__(self, trajectory, variablePoints, costFunc, k):

        self.trajectory = np.copy(trajectory)
        self.variablePoints = variablePoints
        self.costFunc = costFunc
        self.k = k
                                                         
        self.err_best_i = costFunc(trajectory, variablePoints)                                          # best individual error
        pos = trajectory[k].copy()
        self.pos_best_i = pos                                                                           # best individual position

        pos[1] += np.random.normal(0, self.STD_DEV)
        pos[2] += np.random.normal(0, self.STD_DEV)

        self.position_i = pos                                                                           # particle position
        self.err_i = costFunc(trajectory, variablePoints)                                               # individual error
        self.velocity_i = [None, np.random.uniform(-self.VELOCITY, self.VELOCITY), np.random.uniform(-self.VELOCITY, self.VELOCITY), None]  # particle velocity
        # print(f'Trajectory[k]: {trajectory[k]}. New pos: {pos[1:3]}')    



    def evaluate(self):

        self.trajectory[self.k] = np.copy(self.position_i)

        self.err_i = self.costFunc(self.trajectory, self.variablePoints)


        if self.err_i < self.err_best_i:

            # print(f'New local minimum found')
            # print(f'self.err_best_i: {self.err_best_i} -> self.err_i: {self.err_i}')

            self.pos_best_i = np.copy(self.position_i)
            self.err_best_i = self.err_i


    def update_velocity(self, pos_best_g):

        w = .94
        c1 = np.random.random()
        c2 = np.random.random()

        r1 = np.random.random()
        r2 = np.random.random()

        for i in range(1, 3):

            vel_cognitive = c1*r1*(self.pos_best_i[i] - self.position_i[i])
            vel_social = c2*r2*(pos_best_g[i] - self.position_i[i])
            self.velocity_i[i] = w*self.velocity_i[i] + vel_cognitive + vel_social



    def update_position(self):

        for i in range(1, 3):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

    


class PSO:

    def __init__(self, trajectory, variablePoints, numIter, costFunc=aux_CalculateTrajectoryError, num_particles=2000):

        self.trajectory = np.copy(trajectory)
        self.variablePoints = variablePoints
        self.costFunc = costFunc
        self.numIter = numIter
        self.num_particles = num_particles

        self.bestTraj = np.copy(self.trajectory)

        self.pos_best_g = dict()                                                            # best group position
        self.err_best_g = dict()                                                            # best group error


        self.swarm = dict()

    def run(self):
        
        for k in np.where(self.variablePoints)[0]:

            self.swarm[k] = []
            self.pos_best_g[k] = np.copy(self.bestTraj[k])
            self.err_best_g[k] = self.costFunc(self.bestTraj, self.variablePoints)

            for _ in range(self.num_particles):

                self.swarm[k].append(Particle(self.bestTraj, self.variablePoints, self.costFunc, k))

        for _ in range(self.numIter):

            for k in np.where(self.variablePoints)[0]:

                for i in range(self.num_particles):

                    self.swarm[k][i].evaluate()

                    if self.swarm[k][i].err_best_i < self.err_best_g[k]:

                        # print('New global minimum found.')

                        # print(f'self.err_best_g[k]: {self.err_best_g[k]} -> self.swarm[k][i].err_i: {self.swarm[k][i].err_i}')

                        self.pos_best_g[k] = np.copy(self.swarm[k][i].pos_best_i)
                        self.err_best_g[k] = float(self.swarm[k][i].err_best_i)


                    self.swarm[k][i].update_velocity(self.pos_best_g[k])
                    self.swarm[k][i].update_position()

            # for k in np.where(self.variablePoints)[0]:

            #     for i in range(self.num_particles):


            for k in np.where(self.variablePoints)[0]:

                self.bestTraj[k] = self.pos_best_g[k].copy()

        if self.costFunc(self.bestTraj, self.variablePoints) <  self.costFunc(self.trajectory, self.variablePoints):
            return self.bestTraj

        # print('No improved path found.')
        return self.trajectory


def Algorithm_Bee_Colony(trajectory, variablePoints, loss_function=aux_CalculateTrajectoryError, colony_size=200, n_iter=2000, max_trials=20, simulations=5):

    bestTraj = np.copy(trajectory)
    
    for _ in range(simulations):
        
        optimizer = ABC(bestTraj, variablePoints, loss_function, colony_size, n_iter, max_trials)
        
        optimizer.optimize()
        
        
        for k in np.where(variablePoints)[0]:
            bestTraj[k] = np.copy(optimizer.optimality_tracking[k])
            

            
    return bestTraj

@add_metaclass(ABCMeta)
class ArtificialBee:
    
    TRIAL_INITIAL_DEFAULT_VALUE = 0
    INITIAL_DEFAULT_PROBABILITY = 0.0
    STD_DEV = .001
    
    def __init__(self, trajectory, variablePoints, k, loss_function):
        
        self.trajectory = np.copy( trajectory )
        self.variablePoints = variablePoints
        self.k = k
        self.pos = self.trajectory[self.k]
        self.pos[1] += np.random.normal(0, ArtificialBee.STD_DEV)
        self.pos[2] += np.random.normal(0, ArtificialBee.STD_DEV)
        
        self.loss_function = loss_function

        self.fitness = loss_function(self.trajectory, self.variablePoints)
        self.trial = ArtificialBee.TRIAL_INITIAL_DEFAULT_VALUE
        self.prob = ArtificialBee.INITIAL_DEFAULT_PROBABILITY
        
    
    def update_bee(self, pos, fitness):
        
        if fitness <= self.fitness:
            
            self.pos = pos
            self.trajectory[self.k] = self.pos
            self.fitness = fitness
            self.trial = 0
            
        else:
            self.trial += 1
            
    def reset_bee(self, max_trials):
        
        if self.trial >= max_trials:
            
            self.__reset_bee()
            
    def __reset_bee(self):
        
        self.pos = self.trajectory[self.k]
        self.pos[1] += np.random.normal(0, ArtificialBee.STD_DEV)
        self.pos[2] += np.random.normal(0, ArtificialBee.STD_DEV)
        self.fitness = self.loss_function(self.trajectory, self.variablePoints)
        self.trial = ArtificialBee.TRIAL_INITIAL_DEFAULT_VALUE
        self.prob = ArtificialBee.INITIAL_DEFAULT_PROBABILITY

class EmployeeBee(ArtificialBee):
    
    def explore(self, max_trials):
        
        if self.trial <= max_trials:
            
            component = np.copy(self.pos)
            component[1] += np.random.normal(0, ArtificialBee.STD_DEV)
            component[2] += np.random.normal(0, ArtificialBee.STD_DEV)
            phi = np.random.uniform(low=-1, high=1, size=len(self.pos))
            n_pos = self.pos + (self.pos - component) * phi
            tmp = np.copy(self.trajectory)
            tmp[self.k] = n_pos
            n_fitness = self.loss_function(tmp, self.variablePoints)
            self.update_bee(n_pos, n_fitness)
            
    def get_fitness(self):
        return 1 / (1 + self.fitness) if self.fitness >= 0 else 1 + np.abs(self.fitness)
    
    def compute_prob(self, max_fitness):
        self.prob = self.get_fitness() / max_fitness

class OnlookerBee(ArtificialBee):

    STD_DEV_CANDIDATE_EXPLOIT = .000001
    
    def onlook(self, best_food_sources, max_trials):
        candidate = np.random.choice(best_food_sources)
        self.__exploit(candidate.pos, candidate.fitness, max_trials)
    
    
    def __exploit(self, candidate, fitness, max_trials):
        
        if self.trial <= max_trials:
            
            # component = np.random.choice(candidate[1:3])
            phi = np.random.uniform(low=-OnlookerBee.STD_DEV_CANDIDATE_EXPLOIT, high=OnlookerBee.STD_DEV_CANDIDATE_EXPLOIT, size=(len(candidate) - 1))
            STD = np.random.normal(0, OnlookerBee.STD_DEV_CANDIDATE_EXPLOIT)
            n_pos = candidate[:1].tolist() + (np.copy(candidate[1:]) + (np.copy(candidate[1:]) - STD) * phi).tolist()

            tmp = np.copy(self.trajectory)
            tmp[self.k] = np.copy(n_pos)
            n_fitness = self.loss_function(tmp, self.variablePoints)

            if n_fitness < fitness:
                # print(f'phi: {phi}')
                # print(f'STD: {STD}')
                # print(f'Candidate: {candidate}')
                # print(f'n_pos: {n_pos}')
                # print()

                # print(f'n_fitness - fitness')
                # print(f'{n_fitness} - {fitness}')
                # print()

                
                self.pos = np.copy(n_pos) # n_pos
                self.trajectory[self.k] = np.copy(self.pos)
                self.fitness = n_fitness
                self.trial = 0
                # print('EXPLOITED')
                
            else:
                
                self.trial += 1


class ABC:
    
    def __init__(self, trajectory, variablePoints, loss_function, colony_size, n_iter, max_trials):
        
        self.trajectory = np.copy(trajectory)
        self.variablePoints = variablePoints
        self.colony_size = colony_size
        self.loss_function = loss_function
        self.n_iter = n_iter
        self.max_trials = max_trials
        
        self.optimal_solution = dict()
        self.optimality_tracking = dict()
        
        for k in np.where(self.variablePoints)[0]:
            self.optimal_solution[k] = None
            
            self.optimality_tracking[k] = np.copy(self.trajectory[k])
        
        
        self.employee_bees = dict()
        self.onlooker_bees = dict()
        
    
    def __reset_algorithm(self):
        self.optimal_solution = dict()
        self.optimality_tracking = dict()
        
        for k in np.where(self.variablePoints)[0]:
            self.optimal_solution[k] = None
            
            self.optimality_tracking[k] = np.copy(self.trajectory[k])
        
        
        
    def __update_optimality_tracking(self, k):

        Error = aux_CalculateTrajectoryError(self.trajectory, self.variablePoints)
        tmp = np.copy(self.trajectory)
        tmp[k] = np.copy(self.optimal_solution[k].pos)
        error = aux_CalculateTrajectoryError(tmp, self.variablePoints)
        
        if error < Error:
            
            # print(f'For point {k} in trajectory.')
            # print(f'Old error: {Error}')
            # print(f'New error: {error}')
            # print()
            
            self.trajectory = np.copy(tmp)
            
            self.optimality_tracking[k] = np.copy( self.optimal_solution[k].pos )
        
        
    def __update_optimal_solution(self, k):
        n_optimal_solution = min(self.onlooker_bees[k] + self.employee_bees[k], key=lambda bee: bee.fitness)
        
        if not self.optimal_solution[k]:
            
            self.optimal_solution[k] = deepcopy(n_optimal_solution)
        
        else:
            
            if n_optimal_solution.fitness < self.optimal_solution[k].fitness:
                
                # print(f'New optimal solution for {k}: {n_optimal_solution.pos}')
                              
                self.optimal_solution[k] = deepcopy(n_optimal_solution)
    
    
    
    def __initialize_employees(self, k):

        self.employee_bees[k] = []
        
        for _ in range(self.colony_size // 2):
            self.employee_bees[k].append(EmployeeBee(self.trajectory, self.variablePoints, k, self.loss_function))
            
    def __initialize_onlookers(self, k):

        self.onlooker_bees[k] = []
        
        for _ in range(self.colony_size // 2):
            self.onlooker_bees[k].append(OnlookerBee(self.trajectory, self.variablePoints, k, self.loss_function))
            
    def __employee_bees_phase(self, k):
        for _ in map(lambda bee: bee.explore(self.max_trials), self.employee_bees[k]): pass
        
    def __calculate_probabilities(self, k):
        
        sum_fitness = sum(map(lambda bee: bee.get_fitness(), self.employee_bees[k]))
        for _ in map(lambda bee: bee.compute_prob(sum_fitness), self.employee_bees[k]): pass
        
    
    def __select_best_food_sources(self, k):
        
        self.best_food_sources = list(filter(lambda bee: bee.prob > np.random.uniform(low=0, high=1), self.employee_bees[k]))
        
        while not self.best_food_sources:
            
            self.best_food_sources = list(filter(lambda bee: bee.prob > np.random.uniform(low=0, high=1), self.employee_bees[k]))
            
    def __onlooker_bees_phase(self, k):
        for _ in map(lambda bee: bee.onlook(self.best_food_sources, self.max_trials), self.onlooker_bees[k]): pass
        
    def __scout_bees_phase(self, k):
        for _ in map(lambda bee: bee.reset_bee(self.max_trials), self.onlooker_bees[k] + self.employee_bees[k]): pass
        
    def optimize(self):
        
        self.__reset_algorithm()
        
        for k in np.where(self.variablePoints)[0]:

            self.__initialize_employees(k)
            self.__initialize_onlookers(k)
        
        
        for _ in range(self.n_iter):
            
            for k in np.where(self.variablePoints)[0]:
            
                self.__employee_bees_phase(k)
                self.__update_optimal_solution(k)

                self.__calculate_probabilities(k)
                self.__select_best_food_sources(k)

                self.__onlooker_bees_phase(k)
                self.__scout_bees_phase(k)

                self.__update_optimal_solution(k)
                self.__update_optimality_tracking(k)
            

            



def getNeighbours( point, n=1000, STD_DEV=.001 ):
    neighbours = []

    for _ in range( n ):

        neighbour = np.copy( point )

        neighbour[1] += np.random.normal( 0, STD_DEV )
        neighbour[2] += np.random.normal( 0, STD_DEV )

        neighbours.append(neighbour)

    return neighbours