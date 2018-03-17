#==============================================================================
# Initialization-->Fitness assignment-->Selection --> crossover --> mutation
# (fitness > min_value) to be allowed to live. Track corssings allowed as this 
#       will lead to a lower fitness score
# Max 15 joints considered. TODO: Allow variable joint numbers if required
# 
#==============================================================================

import numpy as np
import random
import time

import logging
from os.path import expanduser


from Master_Test_Script_Map import TestRun

#specify results output file
f = open('/home/f1/catkin_ws/src/virtual_racetrack/virtual_roadtest/log/GA_results.txt','w')

#goal is to maximize fitness function
"""Global Varables"""
populationSize = 2
generations = 1
mutationRate = 0.5 # each value has a 20% chance of being chosen to be mutated
NumFeatures = 15 # total number of features. i.e. number of joints

"""Define feature limits... in this case it is joint turning angles"""
maxValues = np.empty(15)
maxValues.fill(3.15)

minValues = np.empty(15)
minValues.fill(-3.15)


f.write('populationSize = ' + str(populationSize) + '\n')
f.write('generations = ' + str(generations)+ '\n')
f.write('___________________________________ \n')


    
"""Create a member of the population."""
def create_individual():
       
    chromosome = np.zeros((15,), dtype=np.int)  

    return chromosome
     
"""Mutation operators have been utilized extensively in
MOEAs as solution variation mechanisms. Mutation
operators assist to the better exploration of the search
space."""

def mutation(child):
#    child = np.copy(parent)
   
    # mutate each feature in child
    for index in range(0, len(child)):
        mutationValue = random.uniform(-0.4,0.4)
        child[index] += mutationValue
        
    # keep feature values within limits
    for f in range(0, len(child)):
        if child[f] > maxValues[f]:
            child[f] = maxValues[f]
        elif child[f] < minValues[f]:
            child[f] = minValues[f]    
    
    return child
  
 
"""Crossover between 2 parents to give 1 child - child is divided in the center """     
def Crossover(parent1, parent2):
    choose = random.uniform(0,1) #choose parent seqience
    child = np.copy(parent1)
    
    if (choose < 0.5):
        # first half = p2, second half = p1
        for elem in range(0, int(len(child) / 2)):
            child[elem] = parent2[elem]
    else:
        # first half = p1, second half = p2
        for elem in range(int(len(child) / 2), len(child)):
            child[elem] = parent2[elem]       
        
    return child
    
#    
#def updateMapSettings(individual):
#    string = ""
#    with open(filename=home+'/catkin_ws/src/gzbo2_generator/Output.txt', "w") as text_file:
#        for fIndex in range(0,NumFeatures):
#            text_file.write(str(individual[fIndex]))
    
def assignJoints(j_array):
    j1 = j_array[0]
    j2 = j_array[1]
    j3 = j_array[2]
    j4 = j_array[3]
    j5 = j_array[4]
    j6 = j_array[5]
    j7 = j_array[6]
    j8 = j_array[7]
    j9 = j_array[8]
    j10 = j_array[9]
    j11 = j_array[10]
    j12 = j_array[11]
    j13 = j_array[12]
    j14 = j_array[13]
    j15 = j_array[14]

    
    return j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15
    
        
    
def main():
    
    home = expanduser("~")
    
    # Initializing the logfile
    logging.basicConfig(filename=home+'/catkin_ws/src/virtual_racetrack/virtual_roadtest/log/test_logs.log', 
                     filemode='w',                        
                     level=logging.DEBUG,
                     format='%(asctime)s:%(levelname)s:%(message)s')
                     

    """ initialize population """
    population = np.zeros((populationSize, NumFeatures))
   
    # loop through each member of population
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
        # loop through each member of population
        for pop_index in range(populationSize):
            #execute master test script
            #return fitness value
            j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15 = assignJoints(population[pop_index])
            fitness[pop_index] = TestRun(j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,j12,j13,j14,j15) #run master script and calculate fitness
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
        new_population = np.zeros((populationSize, NumFeatures))
        
        elites = int(populationSize*0.1)
        if(elites <= 1): # run at least once
            elites = 1
        for i in range(elites):
            print (new_population[i])
            print (population[i])
            new_population[i] = population[i]
            
        f.write('top_10_elite = ' + str(new_population) + '\n')

     
        """ perform crossover """     
        for i in range(elites, populationSize):
            #choose parents        
            p1 = random.randint(0, elites)
            p2 = random.randint(0, elites)
            
            # make sure same parent is not chosen
            while (p2 == p1):
                p2 = random.randint(0, elites)
                
            parent1 = new_population[p1]
            parent2 = new_population[p2]
    
            new_population[i] = Crossover(parent1, parent2)
            
        f.write('after crossover = ' + str(new_population) + '\n\n')

        
        """ perform mutation"""
        for i in range(elites, populationSize):
            if(random.uniform(0,1) < mutationRate):
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