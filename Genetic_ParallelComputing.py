  # -*- coding: utf-8 -*-
"""

Variable threshold model
Consider only one value
Genome length is irrelevant

This is the lastest one

"""

  # -*- coding: utf-8 -*-
"""

@author: leoes
This is to test new algorithm, compute coverage
"""


# Libraries
import pdb #debug with set_trace()
import copy #copy list
import csv #export data
from math import *
import numpy as np
import random
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import multiprocessing as mp

nCpu = mp.cpu_count()


plt.style.use('seaborn-whitegrid')
#plt.style.use('fivethirtyeight')
plt.style.use('seaborn-pastel')
#plt.style.use('seaborn-paper')
#%matplotlib qt
# if sypder is not set to show animation, copy the previous line to the console first then run this code

# Necessary components from Mesa
from mesa import Agent, Model
#from mesa.time import RandomActivation
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
#from mesa.space import SingleGrid


# General setting
#random.seed(a=None, version=2)
#random.seed(time.time())
random.seed(time.time())
#np.random.seed(1)

universalHeight = 10
universalWidth = 10

energyProbability = .5

algaeAppearanceRate = 0.1

generationCount = 2
swarmSize = 10 #number of individuals in each swarm
megaSwarmSize = 10 #number of swarms

genomeLength = 10

#constant
EupThreshold = 25 #Max energy - Eharvest
Ethreshold = 5
Econsume = 2 #walking and sensing
Eharvest = 5
Ereport = 15 #debugging
initialUrgency = 5



maxLifeTimeAlgae = 50 #if algae live longer than this, the robot system fails



EnergyAvailablePercentage = 2
#debug-----------------------------------------------------------------------------



class MoniModel(Model):
    
    def __init__(self, N, width = 10, height = 10):
        self.num_agents = N
        self.width = width
        self.height = height
        self.grid = MultiGrid(height, width, False) #non toroidal grid
        self.schedule = SimultaneousActivation(self)
        
#        self.threshold = [random.randint(0,1) for i in range(N)]
        self.threshold = [random.randint(0,N) for _ in range(N)] #response threshold of N agents in the model, variable
#        self.threshold = [1 for _ in range(N)] #fix
#        self.threshold = [1,2,3,4,5,6,7,8,9,10]
        
        self.abCount = 0 #initial abnormality count
        self.detectedAb = 0
        # Create agent
        
        self.anomalyMap = np.zeros((height,width))
        self.fail = 0
        
        for i in range(self.num_agents):
        
            x = floor(self.width/N*i+self.width/N/2)
#            create and add agent with id number i to the scheduler
            a = MoniAgent(i, self)
            self.schedule.add(a)
            
            #place agent at the center of its limit coor
            self.grid.place_agent(a, (x, 0))
           
#        this part is for visualization only
        self.running = True
        
    def modelEnergy(self):
        sumEnergy = 0
        for agent in self.schedule.agents:
            sumEnergy += agent.energy
        return sumEnergy
        
    def updateAnomaly(self):
# =============================================================================
#         print('self.steps',self.schedule.steps)
#         print(self.anomalyMap)
# =============================================================================
        for i in range(self.height):
            for j in range(self.width):
#                pdb.set_trace()
                
                if self.anomalyMap[i,j] > 0:
                    self.anomalyMap[i,j] += 1
                    if self.anomalyMap[i,j] > maxLifeTimeAlgae:      
                        self.fail = 1
                elif random.random() < algaeAppearanceRate: #rate of appearance of algae ----------------------------------------------------------------------------------------------
                    self.anomalyMap[i,j] += 1

    def show(self):
        '''
        Show genome of all agents in a swarm
        '''
        for agent in self.schedule.agents:
            print(agent.genome)
#            print(agent)
            
    def step(self):
        self.interactionCount = 0
        self.updateAnomaly()
        self.schedule.step() #step of an agent
        
    
    def run_model(self, n):
        '''
        run the model in n step
        '''
        for i in range(n):
#            self.initPos()
            self.step()
#            print(self.schedule.steps)
            
            
            
    def fitness(self):
        while self.fail == 0:
            self.step()
#            print(self.schedule.steps)
        return self.schedule.steps
    
    def intermediateFitness(self): #reset model first
        self.resetModel()
        return self.fitness()
    
    
        
        
    def realFitness(self): #real fitness is average of fitness over many trials (law of large number)
        realFitness = 0
        numberOfTrial = 100
        
        pool = mp.Pool(processes=nCpu)
        results = pool.map(self.intermediateFitness, range(1,numberOfTrial+1))
#        results = [pool.apply(self.intermediateFitness, args=(x,)) for x in range(1,7)]
        pool.close() 
        realFitness = sum(results)/numberOfTrial
        return realFitness
# =============================================================================
#         for i in range(numberOfTrial):
#             self.resetModel()
#             realFitness += self.fitness()
#         realFitness = realFitness/numberOfTrial
#         return realFitness
# =============================================================================
    
    
    def copyModel(self):
        targetModel = MoniModel(self.num_agents, self.width, self.height)
        for agent in targetModel.schedule.agents:
            for originalAgent in self.schedule.agents:
                if (agent.unique_id == originalAgent.unique_id):
                    agent.genome = originalAgent.genome[:]
        return targetModel
        
    def resetModel(self):        #reset steps and fails = 0
        self.schedule.steps = 0
        self.fail = 0
        for i in range(self.height):
            for j in range(self.width):
                self.anomalyMap[i,j] = 0
                
    
class MoniAgent(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.nextPos = (0,0) # careful ----------------------------------------
        self.urgency = initialUrgency
#        self.genome = np.random.randint(1,10,10)
        temp = random.randint(0,10)
        self.genome = [temp for _ in range(genomeLength)]
#        print(self.genome)
        
        self.threshold = self.model.threshold[unique_id]
        
        self.energy = 25 #initial energy

        
        self.regionWidth = ceil(self.model.width/self.model.num_agents) #width of the region to be covered by this agent
        self.originX = floor(self.model.width/self.model.num_agents*unique_id+self.model.width/self.model.num_agents/2)
        self.origin = (self.originX,0)
        
        self.forward = 1 #direction along the spanning tree
        self.commuRange = self.model.width*2/self.model.num_agents
        self.updateRegion = 1 #monitoring region will always  be updated in the first run
        
    def printAgent(self):
        print('pos:',self.pos[0],self.pos[1])
        
         
    def copyAgent(self):
        targetAgent = MoniAgent(self.unique_id,self.model)
        targetAgent.genome = self.genome[:]
        return targetAgent
    
    def confrontOther(self):
#        for agent in self.model.schedule.agent_buffer():
        for agent in self.model.schedule.agents:
            if self != agent:
                if abs(self.pos[0]-agent.pos[0])+abs(self.pos[1]-agent.pos[1]) < 3:
                    self.model.interactionCount += 1
                    return True
                
        return False
        
    def distance(self,pos):
        return sqrt((self.pos[0]-pos[0])**2+(self.pos[1]-pos[1])**2)
        
        
    def move(self):    
        def new_pos(self):
            possible_steps = self.model.grid.get_neighborhood(
            self.pos, moore=True, include_center=False)
            return self.random.choice(possible_steps)
         
        self.nextPos = new_pos(self)
        
            
    def energyAvailable(self):
        #binary model
        if self.model.schedule.steps%10 < EnergyAvailablePercentage: 
            if random.random() < energyProbability:
                return 1
        return 0
        
    def step(self):  
        self.move()
        
        if self.energyAvailable(): #Duration in which an agent could get energy
            self.energy += Eharvest #energy harvested in this step
            if self.energy > EupThreshold:
                self.energy = EupThreshold     
                
                
        if (self.threshold <= self.model.anomalyMap[self.pos[0],self.pos[1]]):    
           
            if self.energy > Econsume:
               self.energy -= Econsume
               self.move()
               
               
               for i in range(self.model.height):
                   for j in range(self.model.width):
                       if self.pos == (i,j) and self.model.anomalyMap[i,j] > 0:
                           if self.energy > Ereport:
                               self.energy -= Ereport
                               self.model.anomalyMap[i,j] = 0
    


    
#    next step after staged change
    def advance(self):
        self.model.grid.move_agent(self, self.nextPos)
        

    
def swarmCrossover(*args): #generate new swarm
    '''
    Generate new swarm from parent swarms
    After parent swarms are decided, random pairs from both parents will be chosen to make new agents in the new swarm
    until the number of offsprings reaches swarm population.
    ''' 
    offspringSwarm = MoniModel(swarmSize,universalWidth,universalHeight)
    parent1 = args[0].threshold
    parent2 = args[1].threshold
    
    for i in range(swarmSize):
        k = random.random()
        if k < 0.49:
            offspringSwarm.threshold[i] = parent1[i]
        elif k < 0.98:
            offspringSwarm.threshold[i] = parent2[i]
        else: 
            offspringSwarm.threshold[i] = random.randint(0,swarmSize)
            

#        print('newIndividual',offspringSwarm.schedule.agents[agent].genome)
    
#    print('after')
#    offspringSwarm.show()
    return offspringSwarm
    

class MegaModel:
    def __init__(self, size):
        self.size = size
        self.megaSwarm = []
        self.sortedFitness = [0 for _ in range(self.size)] #later this is used to hold 
        for _ in range(size):
            model = MoniModel(swarmSize,universalWidth,universalHeight)
            self.megaSwarm.append(model)
            
    def copyMega(self):
        '''
        Create a copy of the mega swarm
        '''
        targetMegaModel = MegaModel(self.size)
        for swarmIndex in range(self.size):
            targetMegaModel.megaSwarm[swarmIndex] = self.megaSwarm[swarmIndex].copyModel()
        return targetMegaModel
    
    
    def nextGeneration(self):
        '''
        Generate a new meta swarm from previous generation
        '''
        fit = [0 for _ in range(self.size)]
        for swarmIndex in range(self.size):
            fit[swarmIndex] = self.megaSwarm[swarmIndex].realFitness() #real fitness instead of fitness
            
#        print('fit',fit)
        self.sortedFitness = sorted(range(len(fit)), key=lambda k: fit[k], reverse = True)
#        print('sorted',sortedFitness)
        megaSwarmCopy = self.copyMega()
        for i in range(megaSwarmSize):
            parent1Index = self.sortedFitness[np.random.randint(0,self.size/2+1)]
            parent2Index = self.sortedFitness[np.random.randint(0,self.size/2+1)]
            parent1 = megaSwarmCopy.megaSwarm[parent1Index]
            parent2 = megaSwarmCopy.megaSwarm[parent2Index]
            self.megaSwarm[i] = swarmCrossover(parent1,parent2)
        
       
    
    def evolve(self, generationCount):
        for _ in range(generationCount):
            self.nextGeneration()
            
#            self.geneDecompose()
#            print('nextGen')
            if self.terminateCondition():
                break
            
    def terminateCondition(self): 
        '''
        Condition for termination of evolution process
        '''
        pass
        return 0
        
    def geneDecompose(self):
        '''
        Generate a histogram to show composition of value of genes
        '''
        flattenGenes = [i for swarm in self.megaSwarm for agent in swarm.schedule.agents for i in agent.genome]
#        plt.figure()
        plt.hist(flattenGenes,bins = (genomeLength +1))
                
        
        
# =============================================================================
# EVOLUTION     
# =============================================================================

superSwarm = MegaModel(megaSwarmSize)
#superSwarm.evolve(generationCount)
for i in range(megaSwarmSize):
    superSwarm.megaSwarm[i].threshold = [swarmSize%(i+1) for _ in range(swarmSize)]


#animation
fig = plt.figure()
def animate(i):
    superSwarm.nextGeneration()
    flattenGenes = superSwarm.megaSwarm[superSwarm.sortedFitness[0]].threshold
#    flattenGenes = [j for swarm in superSwarm.megaSwarm for j in swarm.threshold]
    plt.cla()
    plt.hist(flattenGenes)
    plt.axis([0, swarmSize, 0, swarmSize])
    plt.xlabel('Threshold values')
    plt.ylabel('Frequency')
    
ani = animation.FuncAnimation(fig, animate, interval=5)
plt.show()


