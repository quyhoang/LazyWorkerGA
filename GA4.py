  # -*- coding: utf-8 -*-
"""

@author: hideki
Heuristic method to determine inactivity level of swarms

"""


#lib
import pdb
import copy
import csv
import matplotlib.pyplot as plt
from mesa import Agent, Model
#from mesa.time import RandomActivation
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.space import SingleGrid
from math import *
import numpy as np
import random

#random.seed(a=None, version=2)
#random.seed(1)

#constant
EupThreshold = 25 #Max energy - Eharvest
Ethreshold = 5
Econsume = 2 #walking and sensing
Eharvest = 3
Ereport = 15

EnergyAvailablePercentage = 60


class MoniModel(Model):
    """
    A model for monitoring agents
    """
    def __init__(self, N, width, height):
        
        
        #Monitoring space 
        self.width = width
        self.height = height
        self.grid = MultiGrid(height, width, False) #non toroidal grid
                
        self.schedule = SimultaneousActivation(self)
        
        self.abCount = 0 #initial abnormality count
        self.detectedAb = 0
        self.interactionCount = 0
# =============================================================================
#         self.coveredArea = []

#         self.interactionRateAverage = 0
#         self.coveragePercentage = 0
#         self.coveragePercentageAverage = 0
# =============================================================================
        # Create agents
       
        self.num_agents = N
        for i in range(self.num_agents):
            x = floor(self.width/N*i+self.width/N/2)
#           create and add agent with id number i to the scheduler
            a = MoniAgent(i, self)
            self.schedule.add(a)
            #place agent at the center of its limit coor
            self.grid.place_agent(a, (x, 0))
           
#        this part is for visualization only
#        self.running = True

    def step(self):
#        self.interactionCount = 0
        self.schedule.step()
        

    def run_model(self, n):
        for i in range(n):
#           self.initPos()
            self.step()
#           print(self.schedule.steps)
    
    """
    Calculate fitness of a swarm (success ratio)
    """
    def fitness(self): 
        for _ in range(100):
            self.step()
        if self.abCount == 0:
            self.abCount = 1 #avoid division by 0 error
        return self.detectedAb/self.abCount
    
    
class MoniAgent(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.nextPos = (0,0) # careful ----------------------------------------
        self.sensitivity = 5
        
        self.genome = np.ones(10)*np.random.randint(1,10)
        
        self.energy = 25 #initial energy        
        self.regionWidth = ceil(self.model.width/self.model.num_agents) #width of the region to be covered by this agent
        self.originX = floor(self.model.width/self.model.num_agents*unique_id+self.model.width/self.model.num_agents/2)
        self.origin = (self.originX,0)
        
        self.commuRange = self.model.width*2/self.model.num_agents
#        self.updateRegion = 1 #monitoring region will always  be updated in the first run
#        self.region = [floor(self.model.width/self.model.num_agents*unique_id),floor(self.model.width/self.model.num_agents*(unique_id+1))-1] #range of region of each agent
    
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
                

    def step(self):     
        if (self.model.schedule.steps%100 < EnergyAvailablePercentage):
# =============================================================================
#             print("PrevEnergy: ")
#             print(self.energy)
# =============================================================================
            if self.energy < EupThreshold:
                self.energy += Eharvest #energy harvested in this step
# =============================================================================
#             print("NextEnergy: ")
#             print(self.energy)
# =============================================================================

        if self.confrontOther():
            if self.sensitivity > 0.2: 
                self.sensitivity -= 0.5
        else: 
            if self.sensitivity < 9:
                self.sensitivity += 0.05
            
        if (self.model.schedule.steps%100 > EnergyAvailablePercentage):
             if self.sensitivity < 9:
                 self.sensitivity += 0.2
        else:
             if self.sensitivity > 1:
                 self.sensitivity -= 0.1
                
        if (self.sensitivity > self.genome[self.model.schedule.steps%10]):    
            if random.random() > 0.8: #probability to encounter an abnormality
                self.model.abCount += 1
                if self.energy > Ereport:
                    self.energy -= Ereport
                    self.model.detectedAb += 1
                 #the agent may communicate with other or not, depending on whether there is an abnormality
            
            if self.energy > Econsume:
                self.energy -= Econsume
                self.move()
        
#    next step after staged change
    def advance(self):
        self.model.grid.move_agent(self, self.nextPos)
# =============================================================================
#         if (self.model.schedule.steps == 100):
#             print("fitness")
#             print(self.model.detectedAb/self.model.abCount)
# =============================================================================
#        print(self.pos)
#        print(self.model.schedule.steps)
# =============================================================================


modelList = []

generationCount = 100
genomeLength = 10
swarmSize = 10 #number of individuals in each swarm
swarmPopulation = 10 #number of swarms
mirrorList = [[[0 for x in range(genomeLength)]for y in range(swarmSize)] for z in range(swarmPopulation)]

for i in range(swarmPopulation):
    model = MoniModel(swarmSize,10,10)
#    self.genome = np.ones(10)*np.random.randint(1,10)
    for agent in model.schedule.agents:
        temp = np.random.randint(1,10)
#        agent.genome = [random.uniform(0,10) for _ in range(genomeLength)] #float random
#        agent.genome = [random.randint(0,10) for _ in range(genomeLength)] #int random
        agent.genome = [temp for _ in range(genomeLength)]
        
#        print(agent.genome)
#        print("first gen")
    modelList.append(model) #there will be 5 swarms
    
    for agent in model.schedule.agents:
        mirrorList[i][agent.unique_id]=agent.genome[:] #copy all the genome value into mirrorList
#        print(mirrorList[i][agent.unique_id])
    



# =============================================================================
# import copy
# modelListB = copy.deepcopy(modelList)
# =============================================================================

a = np.zeros(swarmPopulation);
#pdb.set_trace()    
for i in range(swarmPopulation):
    a[i] = modelList[i].fitness()

#print(a)
b = sorted(range(len(a)), key=lambda k: a[k], reverse = True)

#print(b)
    




for i in range(generationCount):
    for j in range(swarmPopulation):
        for k in range(swarmSize): 
            x = np.random.randint(0,swarmPopulation/2+1)
# =============================================================================
#             print("x")
#             print(x)
# =============================================================================
            y = np.random.randint(0,swarmPopulation/2+1)
# =============================================================================
#             print("y")
#             print(y)
# =============================================================================
# =============================================================================
#             x = np.random.randint(0,swarmPopulation)
#             y = np.random.randint(0,swarmPopulation)
# =============================================================================
            m = (modelList[b[x]].random.choice(modelList[b[x]].schedule.agents).genome)
# =============================================================================
#             print("m")
#             print(m)
# =============================================================================
            n = (modelList[b[y]].random.choice(modelList[b[y]].schedule.agents).genome)     
# =============================================================================
#             print("n")
#             print(n)
# =============================================================================
            
            
#            pdb.set_trace()
            l = 0
            for pa,ma in zip(m,n):
                if random.random() < .45:
                    mirrorList[j][k][l] = pa
                elif random.random() < .9:
                    mirrorList[j][k][l] = ma
                else:
                    mirrorList[j][k][l] = random.randint(0,10)
#                   mirrorList[j][k][l] = random.uniform(1,10)       
                l += 1
# =============================================================================
#             print("mirror")
#             print(mirrorList[j][k])
# =============================================================================
    for i1 in range(swarmPopulation):
        for agent in modelList[i1].schedule.agents:
            agent.genome = mirrorList[i1][agent.unique_id][:]
# =============================================================================
#             if i1 == 0 and agent.unique_id == 0:
#                 print("agent.genome")
#                 print(agent.genome)
# =============================================================================
    
    for i2 in range(swarmPopulation):
        a[i2] = modelList[i2].fitness()

#    print(a)
    b = sorted(range(len(a)), key=lambda k: a[k], reverse = True)
    print(b)

    flat = [j for sub in mirrorList[b[0]] for j in sub]
    print(flat)
    plt.hist(flat,bins = 10)
# =============================================================================
# for i in range(generationCount):
#     for j in range(swarmPopulation):
#         for agent in modelListB[j].schedule.agents: 
#             genome1 = modelList[b[1]].random.choice(modelList[b[1]].schedule.agents).genome
# #            print("genome1")
# #            print(j)
# #            print(genome1)
#             genome2 = modelList[b[0]].random.choice(modelList[b[0]].schedule.agents).genome
#             agent.genome = np.array(list(zip(genome1,genome2))).flatten()
#     modelList = modelListB
#     
#     for i in range(5):
#         a[i] = modelList[i].fitness()
# 
# =============================================================================