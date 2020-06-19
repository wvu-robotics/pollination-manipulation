#!/usr/bin/env python

import rospy
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import kinova_msgs.msg

import numpy as np, random, operator, pandas as pd, matplotlib.pyplot as plt



class Fitness:
    def __init__(self, route):
        self.route = route
        self.distance = 0
        self.fitness= 0.0
    def routeDistance(self):
        if self.distance == 0:
            pathDistance = 0
            for i in range(0, len(self.route)):
                fromFlower = self.route[i]
                toFlower = None
                if i + 1 < len(self.route):
                    toFlower = self.route[i + 1]
                else:
                    toFlower = self.route[0]
                pathDistance += fromFlower.distance(toFlower)
            self.distance = pathDistance
        return self.distance

    def routeFitness(self):
        if self.fitness == 0:
            self.fitness = 1.0 / float(self.routeDistance())
        return self.fitness


class Flower: #Will take flower message as input
    def __init__(self, x, y, z, id):
        self.x = x
        self.y = y
        self.z = z
        self.quaternion = np.array([0, 0 , 0, 1])
        self.id = id
        # quaternion distance  = 1 -inner_product(q1,q2)^2
        #quaternion angle = acos (2*inner_product(q1,q2)^2-1)

    def distance(self, flower):
        xDis = abs(self.x - flower.x)
        yDis = abs(self.y - flower.y)
        zDis = abs(self.z - flower.z)
        ##Implement some angle cost
        return np.sqrt(np.power(xDis,2) + np.power(xDis,2) +np.power(xDis,2) ) * (1-np.arccos(2*np.dot(self.quaternion,flower.quaternion)**2-1))

    def __repr__(self):
        return "(" + str(self.id) + ")"


class GA:
    def __init__(self, population, popSize, eliteSize, mutationRate, generations):
        self.pop = self.initialPopulation(popSize, population) # Creates random routes for population
        self.eliteSize = eliteSize
        self.mutationRate = mutationRate
        self.generations = generations
        print("Initial distance: " + str(1 / self.rankRoutes()[0][1]))

        for i in range(0, generations):
            self.nextGeneration()
        print("Final distance: " + str(1 / self.rankRoutes()[0][1]))
        bestRouteIndex = self.rankRoutes()[0][0]
        bestRoute = self.pop[bestRouteIndex]
        print (bestRoute)



    def createRoute(self, flowerList): ##Change for not sampling through all routes ********
        route = random.sample(flowerList, len(flowerList))
        return route

    def initialPopulation(self, popSize, flowerList):
        population = []
        for i in range(0, popSize):
            population.append(self.createRoute(flowerList))
        return population

    def rankRoutes(self):
        fitnessResults = {}
        for i in range(len(self.pop)):
            fitnessResults[i] = Fitness(self.pop[i]).routeFitness()
        return sorted(fitnessResults.items(), key = operator.itemgetter(1), reverse = True)

    def selection(self, ranked):
        selectionResults = []
        df = pd.DataFrame(np.array(ranked), columns=["Index","Fitness"])
        df['cum_sum'] = df.Fitness.cumsum()
        df['cum_perc'] = 100*df.cum_sum/df.Fitness.sum()

        for i in range(self.eliteSize):
            selectionResults.append(ranked[i][0])
        for i in range(len(ranked) - self.eliteSize):
            pick = 100*random.random()
            for i in range(len(ranked)):
                if pick <= df.iat[i,3]:
                    selectionResults.append(ranked[i][0])
                    break
        return selectionResults

    def matingPool(self, selectionResults):
        matingpool = []
        for i in selectionResults:
            matingpool.append(self.pop[i])
        self.pop = matingpool

    def breed(self, parent1, parent2):
        child = []
        childP1 = []
        childP2 = []

        geneA = int(random.random() * len(parent1))
        geneB = int(random.random() * len(parent1))

        startGene = min(geneA, geneB)
        endGene = max(geneA, geneB)

        for i in range(startGene, endGene):
            childP1.append(parent1[i])

        childP2 = [item for item in parent2 if item not in childP1]

        child = childP1 + childP2
        return child


    def breedPopulation(self):
        children = []
        length = len(self.pop) - self.eliteSize
        pool = random.sample(self.pop, len(self.pop))

        for i in range(self.eliteSize):
            children.append(self.pop[i])

        for i in range(length):
            child = self.breed(pool[i], pool[len(self.pop)-i-1])
            children.append(child)

        self.pop = children

    def mutate(self, individual):
        for swapped in range(len(individual)):
            if(random.random() < self.mutationRate):
                swapWith = int(random.random() * len(individual))
                individual[swapped],individual[swapWith] = individual[swapWith],individual[swapped]
        return individual

    def mutatePopulation(self):
        mutatedPop = []
        for ind in range(len(self.pop)):
            mutated_individual_ = self.mutate(self.pop[ind])
            mutatedPop.append(mutated_individual_)
        self.pop = mutatedPop


    def nextGeneration(self):
        popRanked = self.rankRoutes()
        selectionResults = self.selection(popRanked)
        self.matingPool(selectionResults)
        self.breedPopulation()
        self.mutatePopulation()


def main():

    cityList = []

    for i in range(0,25):
            cityList.append(Flower(x=int(random.random() * 200), y=int(random.random() * 200),z=int(random.random() * 200), id= i))
    geneticAlgorithm = GA(population=cityList, popSize=100, eliteSize=20, mutationRate=0.01, generations=100)

if __name__ == '__main__':
    main()
