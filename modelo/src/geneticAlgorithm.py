#!/usr/bin/env python3

import numpy as np
import collections

from robotClass import Robot


class GeneticAlgorithm():

    def __init__(self, NRobots) -> None:
      
        ## =============== Set VARIABLES =============== ## 
        self.NRobots = NRobots

        ## =============== Define CONSTANTS =============== ##
        self.prob_merge = .5
        self.prob_mutate = .05
        self.prob_selectParent = .2
        self.mergeWithMean = False
        self.rangeToMutate = .02
        


    def selectParents(self, allFitnessDict: dict) -> list:
        """Function that return the list with the 2 fitness select

        Args:
            allRobots (dict): _description_

        Returns:
            list: list with the 2 fitness select
        """

        return self.__battleParents(list(allFitnessDict.keys()))
        

    def __battleParents(self, parentsFitness: list) -> list:
        """Function to take two robots to merge for create a child.
            - The solution adopte is the next:
                - All fitness fight with all, and have a certain probability to win
                - Then create a dictionary with the key is each fitness and the 
                    argument is the number of battles win each fitness.
                - Do this we obtain a certain probability that don choose all time the 
                2 best fitness, that is, we have a range to explore and dont only improve

        Args:
            parentsFitness (list): List with all fitness values

        Returns:
            list: List with the two slected fitness
        """
        # Create de dictionay
        dictNumberWinsBattle = {}
        for index, fitnessBattle in enumerate(parentsFitness):
            # Put into dictionary the fitness (key) and initialize the number 
            # of battles win to 0
            dictNumberWinsBattle[fitnessBattle] = 0
            # Delete the element because otherwise it would fight against itself
            
            parentsFitness.pop(index)
            for opponent in parentsFitness:
                # If greatter than the opponent
                if fitnessBattle > opponent:
                    # With a propability of win
                    if np.random.rand(1).round(1)[0] > self.prob_selectParent:
                        dictNumberWinsBattle[fitnessBattle] += 1
                else:
                    if np.random.rand(1).round(1)[0] < self.prob_selectParent:
                        dictNumberWinsBattle[fitnessBattle] += 1
            # then the all fight, insert again the fitness into the list
            parentsFitness.insert(index, fitnessBattle)

        # To finish sort the dictionary by the number of battles win  
        battleWinSort = collections.OrderedDict(sorted(dictNumberWinsBattle.items()))

        # Swap the list because sort from lowest to highest
        # after take the 2  first elements of the keys list
        list2BestBattleWins = list(battleWinSort.keys())[::-1][0:2]

        return list2BestBattleWins


    def merge(self, robot1: Robot, robot2: Robot) -> dict:
        """Function that return a weight dictionary as follows:
                {Sensor1: [weightVelLinear, weightVelAngular]}.
            For do this merge, use a certain probability like as 0.5 for select
            weight form robot1 or robot2

        Args:
            robot1 (Robot): Robot one to merge
            robot2 (Robot): Robot two to merge

        Returns:
            dict: Dictionary weight after merge two robots.
        """
        child = {}
        ### Important
        for nameSensor in robot1.getWeightDict().keys():
            # The probability
            ## MEdia ponderada
            if self.mergeWithMean:
                # print(robot1.getWeightDict()[nameSensor])
                child[nameSensor] = [(robot1.getWeightDict()[nameSensor][0] + robot2.getWeightDict()[nameSensor][0] )/2,
                                    (robot1.getWeightDict()[nameSensor][0] + robot2.getWeightDict()[nameSensor][0] )/2  ]

            else:
                if np.random.rand(1).round(2)[0] < self.prob_merge:
                    ## print(nameSensor, ": ", robot1.getWeightDict()[nameSensor], "padre1")
                    ## print(nameSensor, ": ",robot2.getWeightDict()[nameSensor], "padre2")

                    child[nameSensor] = robot1.getWeightDict()[nameSensor]
                
                else:
                    child[nameSensor] = robot2.getWeightDict()[nameSensor]

    
        return child


    def mutateADN(self, child: dict) -> dict:
        """Function to mutate a child with a probablity.
            For do this, recive a dictionary like as:
                {Sensor1: [weightVelLinear, weightVelAngular],..}.
            
            And for each weight with a very small propability to mutate. 
            For the mutation is used the complementary number.

        Args:
            child (dict): Dictionary with weight for each sensor

        Returns:
            dict: the new child' dcit with mutation made
        """

        ## ========== NEW VERSION ========== ##  For dict
        for nameSensor in child.keys():
            listweightValues = child[nameSensor]
            for i in range(len(child[nameSensor])):
                if np.random.rand(1).round(2)[0] < self.prob_mutate:
                    ### Cuidado value +- 0.2
                    #listweightValues[i] = 1 - listweightValues[i]
                    
                    listweightValues[i] = np.random.uniform(listweightValues[i]-self.rangeToMutate, [listweightValues[i]+self.rangeToMutate if listweightValues[i]+self.rangeToMutate<=1 else listweightValues[i]][0], 1)[0].round(2)
                    

            child[nameSensor] = listweightValues

        return child


    def createalgorithm(self, firstGen=False, listAlldictWeight:list=None):
        """Function that return 2 dictionaries that contain all robots fitness and 
        all robots.
            First check if its first generation, and after create "NRobots" into a list
            - Start the threath for each robot
            - Stay for all robots to finish 

        Args:
            firstGen (bool, optional): If its first generation. Defaults to False.
            dictWeight (dict, optional): In another generation weight dict (after selec-merge-mutate). Defaults to None.

        Returns:
            dict: Dictionary with all robots fitness {robotFitness: robotName, ..}
            dict: Dictionary with all robots weight  {robotName: Robot}
        """
        
        bestVector = 0

        
        if firstGen:
            listRobots = [Robot("robot"+str(i), firstGen=firstGen) for i in range(1, self.NRobots+1)]
            
            [robot.start() for robot in listRobots]
            [robot.join() for robot in listRobots]

        else:
            
            listRobots = [Robot("robot"+str(i+1), firstGen=firstGen, weightDict=listAlldictWeight[i]) for i in range(0, self.NRobots)]
            
            [robot.start() for robot in listRobots]
            [robot.join() for robot in listRobots]
            
        
        dictAllRobotsFitness = {robot.getFitness(): robot.robotName for robot in listRobots}
        dictAllRobots = {robot.robotName: robot for robot in listRobots} ## .getWeightDict()
        
        return dictAllRobotsFitness, dictAllRobots
        
        
