# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 22:17:26 2020

@author: Hideki
"""
#algaeAppearanceRate = 1
mu, sigma = 0, 10
ite = 50
#lista = [0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,0.1,0.15,0.2,0.25]
#lista = [0.005,0.006,0.008,0.01,0.02,0.03,0.04,0.05,0.06,0.07]
lista = [0.01]
for algaeAppearanceRate in lista:
    sum = 0
    b = MoniModel(20,10,10)
#    b.threshold = [10 for i in range(20)]
    b.threshold = np.random.normal(mu, sigma, 20)
    b.threshold = [random.random()*20 for i in range(20)]
    for i in range(ite):
#        print(b.threshold)
        b.resetModel()
        x = b.fitness()
        sum = sum+x
        print(x)
    sum = sum/ite
#    print('rate',algaeAppearanceRate, sum)
    print(sum)
#    print(b.modelEnergy())
#For hypothesis testing
