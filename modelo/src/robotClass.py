#!/usr/bin/env python3

import rospy
import os
import numpy as np
import threading as th
import time
import random

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates



class Robot(th.Thread):

    def __init__(self, robotName: str, firstGen: bool=False, 
                weightDict: dict=None):

        ## =============== Init THREADS =============== ## 
        # Init thread
        th.Thread.__init__(self)


        ## =============== Set VARIABLES =============== ## 
        # Set variables
        self.robotName = robotName
        self.weightDict = weightDict


        ## =============== Define CONSTANTS =============== ##
        # Define name of sensor into a list of strings
        self.sensors = ["SFL1", "SFL2", "SFC1", "SFC2", "SFR1", "SFR2"] #, "SB1", "SB2"
        # Define Network Outputs
        self.networkOutputs = 2
        # Define rate
        self.rate = 10 ## 5 mais baixa
        # Define weight for each apects into the fitness function
        self.vel_factor = .3
        self.linear_factor = .2
        self.distMin_factor = .5
        # Define the life time of robot on seconds 
        self.robotLifeTime = 190
        ###### More life
        ## =============== Define VARIABLES to use =============== ## 
        # Define velocities
        self.velLinear = np.random.randn(1).round(2)[0]
        self.velAngular = np.random.randn(1).round(2)[0]*np.pi ## np.random.randint(4)
        # Define variables to calculate fitness
        self.dictLinearVel = {0: 0}
        self.dictAngularVel = {0: 0}
        self.minDist2Wall = None
        # Define value of fitness
        self.fitness = None
        # Define dictionary of sensors ai: {"sensor1": valueOfSensor1, ...}
        self.dictSensorValues = None

        ## Define last sensor Value
        self.lastDictSensorValues = None
        self.last2DictSensorValues = None

        ## =============== Define VARIABLES ROS =============== ## 
        # Define subscriber sensor Laser Front
        self.subscriberFrontLaser = rospy.Subscriber("/"+self.robotName+"/laser_front/scan", LaserScan, self.__callbackFront)
        # Define subscriber sensor Laser Back !!!!NOT USED!!!!
        ## self.subscriberBack = rospy.Subscriber("/"+self.robotName+"/laser_back/scan", LaserScan, self.__callbackBack)

        ## Subscriber to gazebo/model_states
        self.subscriberGazeboModelState = rospy.Subscriber("/gazebo/model_states", ModelStates, self.__callbackModelState)

        ## Ground Truth

        self.groundTruth = None

        # Define publisher cmd_vel
        self.pub = rospy.Publisher("/"+self.robotName+"/cmd_vel", Twist, queue_size=1)
        # Define rospy rate
        self.rospyRate = rospy.Rate(self.rate)
        # Define type of momevent
        self.movementTwist = Twist()

        ## =============== Comprobe if FIRST GENERATION =============== ## 
        if firstGen: self.__randomWeightDict()
        else:
            self.__setWeightDict(self.weightDict)


    ## =============== RUN FUNCTION =============== ## 
    ## Run function for thread (.start() ejecute this function)  ## 

    def run(self):
        
        # Wait firts for the CALLBACK
        while self.getDictSensorValues() is None:
            self.rospyRate.sleep()
        
        cont = 0
        while cont != self.robotLifeTime:

            # Reset movement of the robot IMPORTANT
            self.movementTwist.linear.x = 0
            self.movementTwist.angular.z = 0
            self.pub.publish(self.movementTwist)

            
            ## Know its moving
            """
            if self.__isMoving():
                ## if self.robotName == "robot14": print("\n ---------------------------Dont Moving---------------------------\n")
                self.__setDictLinearVel(0)
                self.__setDictAngularVel(0)
            """
            """
            if self.groundTruth is not None:
                ## print("")
                ## print(self.robotName, "vel x -> ", abs(round(self.groundTruth["vel"].linear.x, 3)), \
                ##     " angular z -> ", abs(round(self.groundTruth["vel"].angular.z, 3)))
                ## print("")
                if cont < 10:
                    if self.__isNotMovingGroundTruth():
                        # Set and actualizate all values to use in the network
                        print("DontMoving ", self.robotName)
                        self.__setMinDist2Wall(0)
                        self.__setDictLinearVel(0)
                        self.__setDictAngularVel(0)
                        self.movementTwist.linear.x = 0
                        self.movementTwist.angular.z = 0 
                        self.pub.publish(self.movementTwist)

                    else:
                        # Set and actualizate all values to use in the network
                        minValSensorNow = min(list(self.getDictSensorValues().values()))
                        self.__setMinDist2Wall(minValSensorNow)
                        self.__setDictLinearVel(self.velLinear)
                        self.__setDictAngularVel(self.velAngular)

                        # Activate network
                        dictDirections = self.__setNetWork()

                        # Set the angular and linear values to send topic to robot
                        self.movementTwist.linear.x = dictDirections["linear"]
                        self.movementTwist.angular.z = dictDirections["angular"]
                        self.pub.publish(self.movementTwist)
                else:
                    # Set and actualizate all values to use in the network
                    minValSensorNow = min(list(self.getDictSensorValues().values()))
                    self.__setMinDist2Wall(minValSensorNow)
                    self.__setDictLinearVel(self.velLinear)
                    self.__setDictAngularVel(self.velAngular)

                    # Activate network
                    dictDirections = self.__setNetWork()

                    # Set the angular and linear values to send topic to robot
                    self.movementTwist.linear.x = dictDirections["linear"]
                    self.movementTwist.angular.z = dictDirections["angular"]
                    self.pub.publish(self.movementTwist)
            
            else:
            """
            # Set and actualizate all values to use in the network
            minValSensorNow = min(list(self.getDictSensorValues().values()))
            self.__setMinDist2Wall(minValSensorNow)
            self.__setDictLinearVel(self.velLinear)
            self.__setDictAngularVel(self.velAngular)

            # Activate network
            dictDirections = self.__setNetWork()

            # Set the angular and linear values to send topic to robot
            self.movementTwist.linear.x = dictDirections["linear"]
            self.movementTwist.angular.z = dictDirections["angular"]
            self.pub.publish(self.movementTwist)

            cont += 1
            ## time.sleep(1)
            self.rospyRate.sleep()


    ## =============== RRIVATE FUNCTIONS =============== ## 

    ### =============== CALLBACKS FUNCTIONS =============== ### 

    def __callbackFront(self, msg):
        """Function to set the values of each sensor.
            For do this, create a dictionary, its key is the name of each
            sensor, and the argument is the value of the sensor

        Args:
            msg (LaserScan): List with all values of each sensor
        """
        # Values front sensor left to ritght !!! COMPROBE !!!!
        # Facemos un diccionario cos valores cos nomes dos sensores e o valor dese sensor  [0:len(self.sensors)-2]
        dictValuesSensorsFront = {sensorName: value for value in msg.ranges for sensorName in self.sensors}
        ### Know its moving
        
        if self.lastDictSensorValues is None:
            self.lastDictSensorValues = True
        else:
            if self.lastDictSensorValues is True:
                self.lastDictSensorValues = self.dictSensorValues

            else:
                self.last2DictSensorValues = self.lastDictSensorValues
                self.lastDictSensorValues = self.dictSensorValues
        
        ### To know is in front of wall
        ## dictValuesSensorsFront = self.__isFront2Wall(dictValuesSensorsFront, self.sensors)
        ## if self.robotName == "robot14": print("\n Distancias: ", dictValuesSensorsFront, "\n")
        dictValuesSensorsFront = {sensor: 0 if dictValuesSensorsFront[sensor] < .2 else dictValuesSensorsFront[sensor] for sensor in self.sensors}
        self.__setDictSensorValues(dictValuesSensorsFront)
    


    def __callbackBack(self, msg):
        """FOR USE CHANGE THE ELEMENTS OF THE SENSOR LIST

        Args:
            msg (LaserScan): List with all values of back sensors
        """
        dictValuesSensorsBack = {sensorName: value for value in msg.ranges for sensorName in self.sensors[len(self.sensors)-2:len(self.sensors)]}
        if dictValuesSensorsBack is not None:
            self.__setDictSensorValues(dictValuesSensorsBack, sensorFront=False)

    
    def __callbackModelState(self, msg: ModelStates):
        ## print("ENTROOOOOOO GROUND TRUTH CALLBACK", self.robotName)
        ## print("")
        ## Set GROUND TRUTH
        # extrac de robot name index from file
        indexRobot = msg.name.index(self.robotName)
        poseAndOrientation = msg.pose[indexRobot]
        velLinearAndAngular = msg.twist[indexRobot]

        self.groundTruth = {"pose": poseAndOrientation, "vel": velLinearAndAngular}


    ## =============== Ground Truth ============ ##
    def __isNotMovingGroundTruth(self):

        velLinear = self.groundTruth["vel"].linear
        velAngular = self.groundTruth["vel"].angular
        ## and velAngular.y == 0.0 and velAngular.z == 0.0
        linearZero = True if abs(round(velLinear.x, 3)) < 0.003 else False
        angularZero = True if abs(round(velAngular.z, 3)) < 0.003  else False

        if linearZero is True and angularZero is True:
            return True  
        
        else: return False



    ### ============== In front wall =========== ###
    def __isFront2Wall(self, dictValuesSensor, sensorsUsed):

        listValuesSensor = list(dictValuesSensor.values())
        
        firstElement = listValuesSensor[0]
        cont = 0
        for value in listValuesSensor:
            if round(value, 3) == round(firstElement, 3):
                cont += 1

        value2Sensor = .09
        if cont == len(listValuesSensor):
            ## All values are de same in front of wall
            if self.__isMoving():
                dictValuesSensor = {sensorName: value2Sensor for sensorName in sensorsUsed}
            

        return dictValuesSensor


    def __comprobeIsTheSame3(self, sensors, dict, value2Sensor):

        if self.lastDictSensorValues is True or self.last2DictSensorValues is None:
            ## Robot is moving
            return False

        sameValues = 0
        for sensor in sensors:
            if round(self.last2DictSensorValues[sensor], 3) == round(dict[sensor], 3):
                sameValues += 1

        if sameValues == len(self.sensors): 
            cont = 0
            for i in list(self.lastDictSensorValues.values()):
                if i == value2Sensor: cont += 1
            
            if cont == len(list(self.lastDictSensorValues.values())):
                return True
        
        else: return False

    ### =============== Comprobe Robot move ========== ###
    def __isMoving(self):
        if self.lastDictSensorValues is True or self.last2DictSensorValues is None:
            ## Robot is moving
            return False

        sameValues = 0
        for sensor in self.sensors:
            if round(self.dictSensorValues[sensor], 3) == round(self.lastDictSensorValues[sensor], 3):
                if round(self.dictSensorValues[sensor], 3) == round(self.last2DictSensorValues[sensor], 3):
                    sameValues += 1

        if sameValues == len(self.sensors):
            ## All sensor dont change robot dont move
            return True
        
        else:
            ## Robot is moving
            return False

    ### =============== SETS FUNCTIONS =============== ### 

    def __randomWeightDict(self):
        """
            To set weight for each sensor, we create a dictionary that its key
            is the name of sensor and the value for this key will be a list with
            "n" elements begin "n" is value of network outputs, that is,
            {"Sensor1" : [weight4NetworkOutpu1, weight4NetworkOutput2, weight4NetworkOutput"n"], ..}
        """
        # Set weight on the variable
        ### -1 to 1
        """
        a, b = -.2, .2
        ## value = (b - a)*np.random.rand() + a

        c, d = 0, .5
        ## value2 = (d - c)*np.random.rand() + c
        ## np.random.rand(1).round(2)[0]
        # To linear only positive values
        # To angular both
        self.weightDict = {}

        for sensor in self.sensors:
            value = (b - a)*np.random.rand() + a
            value2 = (d - c)*np.random.rand() + c
            self.weightDict[sensor] = [round(value2, 2) if i == 0 else round(value, 2) for i in range(self.networkOutputs) ]
        """
        self.weightDict = {sensor: [round(random.uniform(0, 1), 2) if i == 0 else round(random.uniform(-1, 1), 2) for i in range(self.networkOutputs) ] for sensor in self.sensors}
        
    def __setMinDist2Wall(self, value: float):
        """
            Function to set the min value of the distance to wall

        Args:
            value (float): Value to check if the distance minimun to wall
        """
        # Comprobe if first time 
        if self.minDist2Wall is None: self.minDist2Wall = value

        # Without comprobe the "value" is smaller than the last minimun value
        else: 
            if self.minDist2Wall > value: self.minDist2Wall = value
        
    def __setMinDist2Wall2(self, value: float):

        if self.minDist2Wall is None:
            self.minDist2Wall = {1: value}

        else:
            times = self.minDist2Wall.keys()[0]
            valueLast = self.minDist2Wall[times]
            self.minDist2Wall = {times+1, valueLast+value}

    def __setDictLinearVel(self, value: float):
        """
            Funtion set new value of linear vel. For do this, we create
            a dictionary that its key is the number of samples and its argument
            is a sumatori of the velocities. For example:
                {20: 1000}
            That is, 20 samples and sumatory of velocities is 1000. Now for 
            calculate the mean is very easy since only we do the division of
            the sumatory by the number of samples, that is, argument / key

        Args:
            value (float): value of liner velocity right now
        """

        numberOfSamples = list(self.dictLinearVel.keys())[0]
        sumAllValues = self.dictLinearVel[numberOfSamples]
        # Only comprobe if the first time
        if numberOfSamples is None: numberOfSamples = 0
        if sumAllValues is None: sumAllValues = 0
        # Sum one more to the samples
        numberOfSamples += 1 

        # Set on the variable
        self.dictLinearVel = {numberOfSamples: sumAllValues+value}

    def __setDictAngularVel(self, value: float):
        """
            Funtion set new value of angular vel. For do this, we create
            a dictionary that its key is the number of samples and its argument
            is a sumatori of the velocities. For example:
                {20: 1000}
            That is, 20 samples and sumatory of velocities is 1000. Now for 
            calculate the mean is very easy since only we do the division of
            the sumatory by the number of samples, that is, argument / key

        Args:
            value (float): value of angular velocity right now
        """
        numberOfSamples = list(self.dictAngularVel.keys())[0]
        sumAllValues = self.dictAngularVel[numberOfSamples]

        # Only comprobe if the first time
        if numberOfSamples is None: numberOfSamples = 0
        if sumAllValues is None: sumAllValues = 0
        # Sum one more to the samples
        numberOfSamples += 1 
        
        # Set on the variable
        self.dictAngularVel = {numberOfSamples: sumAllValues+value}

    def __setDictSensorValues(self, dictSensor: dict, sensorFront=True):
        """
            Funtion to set the value of self.dictSensorValues.

        Args:
            dictSensor (dict): diccionary with key name of sensor and the 
                                argument the value of sensor
            sensorFront (bool, optional): option to know is the front sensor. Defaults to True.
        """
        # Only comprobe if front sensor
        if sensorFront:
            self.dictSensorValues = dictSensor

        else:
            ## self.dictSensorValues = dict(self.dictSensorValues, **dictSensor)
            try:
                for sensor in self.sensors[len(self.sensors)-2:len(self.sensors)]:
                    self.dictSensorValues[sensor] = dictSensor[sensor]
            except:
                for sensor in self.sensors[len(self.sensors)-2:len(self.sensors)]:
                    self.dictSensorValues[sensor] = 0

    def __setNetWork(self) -> dict:
        """
            - Function to set the NetWork with the weight of the sensor and its values
            with output equal a self.networkOutput
            - In the network only do the multiplication value of each sensor by
            respective weight. And sum all result to set the final value for 
            linear velocity and angular velocity.
            - In this network dont use ReLu activate function because must have 
            negative values the output  
            - Activation Function ReLu max(0, arrayWeightValues2Sensor[0])

        Returns:
            dict: Dictionary with len 2, and keys "linear" and "angular", return this velocities
        """

        outputLinear = 0
        outputAngular = 0

        #### Normalizar entre 10
        for sensor in self.sensors:
            # take the values of sensor and each weight
            sensorValue = self.getDictSensorValues()[sensor]
            ## Normalize Sensor value to 0, 1
            sensorValueNormal = sensorValue /10
            arrayWeightValues2Sensor = self.getWeightDict()[sensor]

            outputLinear += sensorValueNormal*arrayWeightValues2Sensor[0]
            outputAngular += sensorValueNormal*arrayWeightValues2Sensor[1]
        
        self.velLinear = outputLinear
        self.velAngular = outputAngular
        return {"linear": outputLinear, "angular": outputAngular}

    def __setWeightDict(self, weightDict: dict):
        """Function only for set the dictionary of weight values

        Args:
            weightDict (dict): Dictionary with key name of sensor and arguments
                        list of the weight for each network output
        """
        self.weightDict = weightDict


    ## =============== PUBLIC FUNCTIONS =============== ## 

    def getMeanLinearVel(self) -> float:
        """Function to return the mean of the linear velocity

        Returns:
            float: mean of the linear velocity
        """
        try:
            allSamples = list(self.dictLinearVel.keys())[0]
            return self.dictLinearVel[allSamples] / allSamples
        except:
            return 0

    def getMeanAngularVel(self) -> float:
        """Function to return the mean of the angular velocity

        Returns:
            float: mean of the angular velocity
        """
        try:
            allSamples = list(self.dictAngularVel.keys())[0]
            return self.dictAngularVel[allSamples] / allSamples
        except:
            return 0

    def getMinDistWithWall(self) -> float:
        """Function that return the minimun valur of the distance to a wall

        Returns:
            float: The min distance to a wall
        """
        return self.minDist2Wall

    def getMinDisWithWall2(self) -> float:

        times = self.minDist2Wall.keys()[0]
        value = self.minDist2Wall[times]
        try:
            return value/times
        
        except:
            return 0


    def getDictSensorValues(self) -> dict:
        """Function that return the dictionary with values of each sensor

        Returns:
            dict: Dictionary with values of each sensor
        """
        return self.dictSensorValues

    def getWeightDict(self) -> dict:
        """Function that return the weight dictionary for each sensor

        Returns:
            _type_: The weight dictionary for each sensor
        """
        return self.weightDict

    ## =============== FITNESS FUNCTION =============== ## 
    def getFitness(self) -> float:
        """Function FITNESS that return the score of the final fitness
            The function takes into account 3 aspects:
                - The mean liear velocity (VELOCITY)
                - The mean Angular velocity (LINEAR) 
                - The min distance to wall (DONT CRASH)
        Returns:
            float: The fitness score
        """
        
        vel_score = self.getMeanLinearVel()*self.vel_factor 
        linear_score = (1 - abs(self.getMeanAngularVel()))*self.linear_factor
        dist_min_score = (self.getMinDistWithWall()/10)*self.distMin_factor
        print(self.robotName, "{")
        print("      Velocidad Media: ", vel_score, "Dont Have Factor -> ", self.getMeanLinearVel())
        print("      Linearidad Media: ", linear_score, "Dont Have Factor -> ", self.getMeanAngularVel())
        print("      Distancia obstaculo: ", dist_min_score, "Dont Have Factor -> ", self.getMinDistWithWall()/10)
        print("      Fitness: ", round(vel_score + linear_score + dist_min_score, 3), " }")
        print("")


        ## linear_score == 0 !!

        ## if vel_score == 0 or dist_min_score == 0:
        ##     return 0

        ## abs(vel_score) + abs(linear_score) + abs(dist_min_score)
        ## 1 - abs(linear_score) \\ 1/ abs(linear_score)
        ## mmirar pesos (cuidado)
        return round(vel_score + linear_score + dist_min_score, 3)

