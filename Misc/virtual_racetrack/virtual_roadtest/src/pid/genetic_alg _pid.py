#==============================================================================
# Initialization-->Fitness assignment-->Selection --> crossover --> mutation
# (fitness > min_value) to be allowed to live. No crashes allowed
#==============================================================================

import numpy as np
import random
import os
import time

import logging
from os.path import expanduser


from Master_Test_Script_PID import TestRun

#specify results output file
f = open('/home/f1/catkin_ws/src/virtual_racetrack/virtual_roadtest/log/GA_results.txt','w')

#goal is to maximize fitness function
"""Global Varables"""
populationSize = 2
generations = 1

f.write('populationSize = ' + str(populationSize) + '\n')
f.write('generations = ' + str(generations)+ '\n')
f.write('___________________________________ \n')


    
"""Create a member of the population."""
def create_individual():

     p = random.uniform(0, 10)
     i = random.uniform(0, 10)
     d = random.uniform(0, 1)
     
     chromosome = np.array([p,i,d])         
     return chromosome
     
"""Mutation operators have been utilized extensively in
MOEAs as solution variation mechanisms. Mutation
operators assist to the better exploration of the search
space."""

def mutation(parent):
    mutationValue_p = random.uniform(-0.3,0.3)
    mutationValue_i = random.uniform(-0.2,0.2)
    mutationValue_d = random.uniform(-0.1,0.1)

    child = np.copy(parent)
    child[0] += mutationValue_p
    child[1] += mutationValue_i
    child[2] += mutationValue_d
    
    return child
  
 
# TODO: 
"""elite perservation """

"""Crossover between 2 parents to give 1 child"""     
def Crossover(parent1, parent2):
    choose = random.randint(0,1) #choose dominant parent
    child = np.copy(parent1)
    if (choose == 0):
        child[0] = parent1[0]
        child[1] = parent2[1]
        child[2] = parent1[2]    
    else:
        child[0] = parent2[0]
        child[1] = parent1[1]
        child[2] = parent2[2]
        
    return child
    
def main():
    
    home = expanduser("~")
    
    # Initializing the logfile
    logging.basicConfig(filename=home+'/catkin_ws/src/virtual_racetrack/virtual_roadtest/log/test_logs.log', 
                     filemode='w',                        
                     level=logging.DEBUG,
                     format='%(asctime)s:%(levelname)s:%(message)s')
                     

    """ initialize population """
    population = np.zeros((populationSize, 3))
   
    for i in range(0,populationSize):
        population[i] = create_individual()
        
    f.write('population = ' + str(population) + '\n')

    
    """Loop through generations"""
        
    for ii in range(1, generations+1):
        logging.info('#######' )
        logging.info('# Generation-->  ' + str(ii))
        logging.info('#######' )
        
        f.write('Generation = ' + str(ii) + '\n\n')


        fitness = np.zeros(populationSize)
        f.write('fitness = ' + str(fitness) + '\n')

        time.sleep(4) # give time for gazebo to shutdown


        """execute test runs and create fitness array - will take longest time to run"""
        for pop_index in range(populationSize):
            #execute master test script
            #TestRun()
            #return fitness value
            fitness[pop_index] = TestRun() #run master script and calculate fitness
            time.sleep(2)

            
        f.write('fitness = ' + str(fitness) + '\n')

        """ Analyze population """    
        #sort population by fitness value        
        for i in range(populationSize):
            max_index = i
            for j in range(i+1, populationSize):
                if(fitness[i] > fitness[max_index]):
                    max_index = j
            fitness[i], fitness[max_index] = fitness[max_index], fitness[i]
            population[i], population[max_index] = population[max_index], population[i]
            
        f.write('Analyze population' + '\n')

        #check
        f.write('population_sorted_by_fitness = ' + str(population) + '\n')

        """ perform selection """  
        # keep the top 10% only 
        new_population = np.zeros((populationSize, 3))
        
        elites = int(populationSize*0.1)
        if(elites <= 1):
            elites = 1
        for i in range(elites):
            print (new_population[i])
            print (population[i])
            new_population[i][0] = population[i][0]
            new_population[i][1] = population[i][1]
            new_population[i][2] = population[i][2]
            
        f.write('top_10_elite = ' + str(new_population) + '\n')

     
        """ perform crossover """     
        for i in range(elites, populationSize):
            #choose parents        
            p1 = random.randint(0, elites)
            p2 = random.randint(0, elites)
            while (p2 == p1):
                p2 = random.randint(0, elites)
                
            parent1 = new_population[p1]
            parent2 = new_population[p2]
    
            new_population[i] = Crossover(parent1, parent2)
            
        f.write('after crossover = ' + str(new_population) + '\n\n')

        
        """ perform mutation"""
        for i in range(elites, populationSize):
            new_population[i] = mutation(new_population[i])
            
        f.write('after mutation = ' + str(new_population) + '\n\n')


        """reset/ update datastructures """
        population = np.copy(new_population)
        new_population = np.zeros(populationSize)
    
    """final results at end of last generation"""
    max_fitness = np.nanmax(fitness)
    max_index = np.argmax(fitness)
    final_value = population[max_index]
    
    print("max fitness = ",max_fitness)
    print("final value = ",final_value)
    
    # close output results file
    f.write('max_fitness = ' + str(max_fitness))
    f.write('final_vale = ' + str(final_value))
    f.close() 
    
if __name__ == '__main__':
        main()
        
        
  
        
        
        
        
        
        
        
        
# TODO:   
"""Polynomial Mutation (PLM)
An Elitist Polynomial Mutation Operator for
improved performance of MOEAs in
Computer Networks - http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6614105
"""
def PLM():
    rand = random.uniform(0,1)
    if (rand <= mutation_prob):
        stdev1 = (xp-xl)/(xu-xl)
        stdev2 = (xp-xp)/(xu-xl)
        
        r = random.uniform(0,1)
        
        if(r<=0.5):
            d = ((2*r)+(1-2*r)(1-d1)^(n_m+1))^(1/(n_m+1))
        else: 
            d = 1-(((2*r)+(1-2*r)(1-d1)^(n_m+1))^(1/(n_m+1)))  