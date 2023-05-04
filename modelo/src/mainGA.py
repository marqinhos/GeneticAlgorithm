#!/usr/bin/env python3

import rospy
import os
import time
import datetime
import collections
import json

from geneticAlgorithm import GeneticAlgorithm

HAVE_OBS = True

if HAVE_OBS is True:
    from recordOBS import OBS


class Generation():

    def __init__(self, first: int=1, startRecord: int=50, stopRecord: int=150, changeProb=30) -> None:
        self.first = first
        self.startRecord = startRecord
        self.stopRecord = stopRecord
        self.changeProb = changeProb
        self.dict = {"first": self.first, "startRecord": self.startRecord, 
                    "stopRecord": self.stopRecord, "changeProb": self.changeProb}
    
    def __call__(self, arg: str) -> int:
        """
            Commands:- "first" - "startRecord" - "stopRecord" - "changeProb"
        Returns:
            int: Value of the command
        """
        return self.dict[arg]


class Replace():

    def __init__(self, nRobots, nReplace=20):
        self.nRobots = nRobots
        self.nReplace = nReplace

    def replace(self, agr: dict) -> list:
        return agr["parents"][0:self.nRobots-self.nReplace] + agr["childs"][0:self.nReplace]



def pprint(nGeneration, bestFitness, meanFiness, nameRobot, time2FirstGen):
    print("==================================================")
    print(" |              XERACION ", nGeneration, " "*(21 - len(str(nGeneration))), "|")
    print(" |          MEAN FITNESS -> ", round(meanFiness, 2), " "*(18 - len(str(round(meanFiness, 2)))), "|")
    print(" |          BEST FITNESS -> ", round(bestFitness, 2), " "*(18 - len(str(round(bestFitness, 2)))), "|")
    print(" |          ROBOT -> ", nameRobot, " "*(25 - len(nameRobot)), "|") 
    print(" |          SINCE 1º GEN -> ", round(time2FirstGen, 2), "s", " "*(18 - len(str(round(time2FirstGen, 2)))- 1), "|")
    print("==================================================")
    print("\n")



if __name__ == "__main__":

    print("            ||||--Genetic Algorithm--||||")

    ## ============== Init CONSTANTS ============== ##
    NROBOTS = 21

    rospy.init_node("genticAlgorithm")
    rate = rospy.Rate(10)
    timeStartGeneration = datetime.datetime.now()
    
    ## ============== Init CLASS ============== ##
    classGeneticAlgorithm = GeneticAlgorithm(NROBOTS)
    if HAVE_OBS is True:
        myOBS = OBS()
    gen = Generation()
    swapRobtos = Replace(NROBOTS)
    ## ============== Define variables ============== ##
    nGeneration = 1
    dictAllBestFitnessAllGen = {}
    dictAllMeanFitnessAllGen = {}
    dictAll2BestWeightAllGen = {}

    while not rospy.is_shutdown() or nGeneration == gen.stopRecord - 4:

        ## ============== Reset simulation for each generation ============== ##
        os.system("rosservice call /gazebo/reset_simulation") 

        ## ============== Screen recorder ============== ##
        if HAVE_OBS is True:
            if nGeneration == gen("startRecord"):
                myOBS.connectOBS()
                myOBS.startRecord()

            if nGeneration == gen("stopRecord"):
                myOBS.stopRecord()
                myOBS.disconnectOBS()
            
        
        ## ============== Run Genetic Algorithm ============== ##
        # First Generation
        if nGeneration == gen("first"):
            print("Entro")
            dictAllRobotsFitness, dictAllRobotsClass = classGeneticAlgorithm.createalgorithm(firstGen=True)
        else:

            dictAllRobotsFitness, dictAllRobotsClass = classGeneticAlgorithm.createalgorithm(listAlldictWeight=listdictWeightChildFinal)

        ## ============== Select 2 best fitness parents ============== ##
        #### ===================== NOT USE KNOW ================== ###
        ## fitnessOrder = collections.OrderedDict(sorted(dictAllRobotsFitness.items()))
        ## =============== SELECT PARENTS ============== #####
        ### ========= the best parents ============== ###
        ## list2BestFitness = list(fitnessOrder.keys())[::-1][0:2]
        ## bestFit = list(fitnessOrder.keys())[::-1][0]
        ## bestFit = list(a.keys()).sort()[::-1][0]
        ## nameRobotbestFit = a[bestFit]
        ## bestWeight = b[nameRobotbestFit]
        
        ## =================== PRINT FITNESS ALL ROBOTS =========== ##
        pretty = json.dumps(dictAllRobotsFitness, indent=4)
        print(pretty)
        ## ============== CHANGE PROBABILITIES AROUND GENERATIONS ============== ##
        ## FIRST GENERATIONS MORE EXPLORE AND MORE MUTATION
        if nGeneration < gen("changeProb"):
            classGeneticAlgorithm.prob_selectParent = .4
            classGeneticAlgorithm.prob_mutate = .4
            classGeneticAlgorithm.prob_merge = .6

        
        else: 
            classGeneticAlgorithm.prob_selectParent = .2
            classGeneticAlgorithm.prob_mutate = .1
            classGeneticAlgorithm.prob_merge = .95
            swapRobtos.nReplace = 18
            ## Do the mean to 2 parents
            classGeneticAlgorithm.mergeWithMean = True



        ## ============== SELECT 2 PARENTS for Battle Method ============== ##
        list2BestFitness = classGeneticAlgorithm.selectParents(dictAllRobotsFitness)
        # Select name of 2 robots as parents 
        list2NameBestParentsFit = [dictAllRobotsFitness[list2BestFitness[0]], dictAllRobotsFitness[list2BestFitness[1]]]
        # Select 2 robot class
        list2BestRobotClass = [dictAllRobotsClass[list2NameBestParentsFit[0]], dictAllRobotsClass[list2NameBestParentsFit[1]]]


        ## ============== MERGE 2 PARENTS ============== ##
        ## Ahora hacemos merge de los padres para crear al hijo
        dictWeightChild = [classGeneticAlgorithm.merge(list2BestRobotClass[0], list2BestRobotClass[1]) for _ in range(NROBOTS)]


        ## ============== MUTATE CHILD ============== ##
        ## Despues lo pasamos por la funcion de mutacion
        ## listFitnessSort = list(dictAllRobotsFitness.keys()).sort()[::-1]

        ## ============ DONT REPLACE ALL PARENTS ============== ##
        listFitnessSort = list(collections.OrderedDict(sorted(dictAllRobotsFitness.items())).keys())

        listRobotClassSort = [dictAllRobotsClass[dictAllRobotsFitness[listFitnessSort[i]]] for i in range(len(listFitnessSort))]
        parentDictWeightSort = [{nameSensor: robot.getWeightDict()[nameSensor] for nameSensor in robot.getWeightDict().keys()} for robot in listRobotClassSort]

        listdictWeightChildFinal = [classGeneticAlgorithm.mutateADN(dictWeightChild[i]) for i in range(len(dictWeightChild))]
        listdictWeightChildFinal = swapRobtos.replace({"parents": parentDictWeightSort, "childs": listdictWeightChildFinal})

        ## ============== PPRINT ON SHELL ============== ##
        timeNext = datetime.datetime.now()
        deltaTime = timeNext - timeStartGeneration
        pprint(nGeneration, list2BestFitness[0], sum(list(dictAllRobotsFitness.keys()))/len(dictAllRobotsFitness.keys()), list2NameBestParentsFit[0], deltaTime.total_seconds())

        ## ============= SAVE DATA =============== ## 
        dictAllBestFitnessAllGen[nGeneration] = list2BestFitness[0]
        dictAllMeanFitnessAllGen[nGeneration] = sum(list(dictAllRobotsFitness.keys()))/len(dictAllRobotsFitness.keys())
        dictAll2BestWeightAllGen[nGeneration] = parentDictWeightSort[0]

        with open(".data/dataWeight.json", "w") as outfile1:
            json.dump(dictAll2BestWeightAllGen, outfile1)

        with open(".data/dataBestFitnessGen.json", "w") as outfile2:
            json.dump(dictAllBestFitnessAllGen, outfile2)

        with open(".data/dataMeanFitnessGen.json", "w") as outfile:
            json.dump(dictAllMeanFitnessAllGen, outfile)

        nGeneration += 1
        rate.sleep()

