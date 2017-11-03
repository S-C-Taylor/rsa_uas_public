#!/usr/bin/env python

import sys
import time


import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

COMMAND_PERIOD = 50 #ms

from status import Status


class Controller(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand = rospy.Publisher('/ardrone/land', Empty)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        self.pubReset = rospy.Publisher('/ardrone/reset', Empty)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel', Twist)
        self.pubPIDCommand = rospy.Publisher('merge/cmd_vel', Twist)

        #self.subPID = rospy.Subscriber('/PID/cmd_vel', Twist, self.receivePID)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD / 1000.0), self.SendCommand)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

    def ReceiveNavdata(self, navdata):
        # Although there is a lot of data in this packet, we're only interested in the state at the moment
        self.status = navdata.state
	#print("We are getting navadata: " + str(self.status))

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
        if (self.status == Status.Landed):
            self.pubTakeoff.publish(Empty())


    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self.pubLand.publish(Empty())


    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
	print("Found emergency")
        self.pubReset.publish(Empty())


    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x = pitch
        self.command.linear.y = roll
        self.command.linear.z = z_velocity
        self.command.angular.z = yaw_velocity

    def SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.status == Status.Flying or self.status == Status.GotoHover or self.status == Status.Hovering:
            #self.pubCommand.publish(self.command)
            self.pubPIDCommand.publish(self.command)

    #def receivePID(self, vel):
        #self.pubCommand(vel)
