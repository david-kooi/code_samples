#!/usr/bin/env python

## Written by David Kooi

import rospy
import actionlib

# ROS Actions
from movex.msg import movementAction, movementGoal,\
                      movementFeedback, movementResult 

# Sambuca Command messages
from movex.srv import Grab, Lift, Move, Turn

from sambuca_util import enum
from nodes.services.http_service import SambucaCmdNames
from definitions.sambuca_settings import SambucaActionServerName


# Define some organizational structures
grab_request = rospy.ServiceProxy(SambucaCmdNames.GRAB, Grab)
lift_request = rospy.ServiceProxy(SambucaCmdNames.LIFT, Lift)
move_request = rospy.ServiceProxy(SambucaCmdNames.MOVE, Move)
turn_request = rospy.ServiceProxy(SambucaCmdNames.TURN, Turn)

# Dictionary of service requests
SambucaCmdFunc = {SambucaCmdNames.MOVE:move_request,
                  SambucaCmdNames.LIFT:lift_request,
                  SambucaCmdNames.GRAB:grab_request,
                  SambucaCmdNames.TURN:turn_request}

class SambucaActionServer(object):
    _feedback = movementFeedback()
    _result   = movementResult()

    def __init__(self, name):

        self.actionName = name

        self.grabRunning = False
        self.turnRunning = False
        self.moveRunning = False
        self.liftRunning = False


        self.server = actionlib.SimpleActionServer(self.actionName,\
                movementAction, execute_cb=self.executeCallback,\
                auto_start=False)

        self.server.start()

    # Timer Callbacks
    def stopGrab(self, event):
        rospy.logdebug("Grab stop received")
        grab_request(0)
        self.grabRunning = False  

    def stopTurn(self, event):
        rospy.logdebug("Turn stop received")
        turn_request(0)
        self.turnRunning = False

    def stopMove(self, event):
        rospy.logdebug("Move stop received")
        move_request(0)
        self.moveRunning = False

    def stopLift(self, event):
        rospy.logdebug("Lift stop received")
        lift_request(0)
        self.liftRunning = False


    def handleRequest(self, cmdName, request, stopMethod):
        rospy.logdebug("Received request: {}".format(cmdName)) 
        rospy.logdebug("Duration: {}".format(request.duration)) 
        rospy.logdebug("Magnitude: {}".format(request.magnitude)) 

        # Do not run with invalid parameters
        if(request.duration == 0):
            rospy.logwarning("Received action with duration 0")
            return


        # Get reference to the service proxy
        # And initiate the action 
        serviceFunction = SambucaCmdFunc[cmdName]
        serviceFunction(request.magnitude) 

        # Run a timer callback to stop the action 
        duration = rospy.Duration(request.duration)
        rospy.Timer(period=duration, callback=stopMethod, oneshot=True)


    def executeCallback(self, goal):
        print("EXECUTE")
        rospy.logdebug("ActionServer: Goal received from :\
                        {}".format(goal.requester)) 


        # Handle grab request
        if(goal.grabRequest.isActive):
            request = goal.grabRequest
            self.handleRequest(SambucaCmdNames.GRAB, request, self.stopGrab) 

            self.grabRunning = True

        # Handle turn request
        if(goal.turnRequest.isActive):
            request = goal.turnRequest
            self.handleRequest(SambucaCmdNames.TURN, request, self.stopTurn) 

            self.turnRunning = True 

        # Handle move request
        if(goal.moveRequest.isActive):
            request = goal.moveRequest
            self.handleRequest(SambucaCmdNames.MOVE, request, self.stopMove)

            self.moveRunning = True
            

        # Handle lift request
        if(goal.liftRequest.isActive):
            request = goal.liftRequest
            self.handleRequest(SambucaCmdNames.LIFT, request, self.stopLift)

            self.liftRunning = True
             

        # Spin while all actions finish
        rospy.logdebug("Waiting for actions to finish")
        runningList = [self.grabRunning, self.turnRunning,
                       self.liftRunning, self.moveRunning]

        # Assume none are stopped 
        allStopped = False 
        while(not allStopped):
            runningList = [self.grabRunning, self.turnRunning,
                           self.liftRunning, self.moveRunning]

            # Spin until all are not running

            # When isRunning is False for all in runningList
            # all(not isRunning...) will return True
            allStopped = all([not isRunning for isRunning in runningList])  




        # Make sure all flags are false for next request
        self.grabRunning = False
        self.turnRunning = False
        self.moveRunning = False
        self.liftRunning = False


        self.server.set_succeeded()
        

        
        
if __name__ == '__main__':
 
    rospy.init_node(SambucaActionServerName, log_level=rospy.DEBUG)
    server = SambucaActionServer(SambucaActionServerName)
    rospy.spin()
