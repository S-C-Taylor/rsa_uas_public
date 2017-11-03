#!/usr/bin/env python

import sys
import time
import os


import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from rsa_uas.msg import Tracker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32

from controller import Controller
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key.Key_E
    PitchBackward    = QtCore.Qt.Key.Key_D
    RollLeft         = QtCore.Qt.Key.Key_S
    RollRight        = QtCore.Qt.Key.Key_F
    YawLeft          = QtCore.Qt.Key.Key_W
    YawRight         = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff          = QtCore.Qt.Key.Key_Y
    Land             = QtCore.Qt.Key.Key_H
    Emergency        = QtCore.Qt.Key.Key_Space
    Up 		 = QtCore.Qt.Key.Key_I
    Down 		 = QtCore.Qt.Key.Key_K
    Left 		 = QtCore.Qt.Key.Key_J
    Right 		 = QtCore.Qt.Key.Key_L
    Clockwise 	 = QtCore.Qt.Key.Key_O
    AntiClockwise    = QtCore.Qt.Key.Key_U



X_THRESH_HOLD = 150
Y_THRESH_HOLD = 150
Z_TRHESH_HOLD = 500
HEADING_THRESHOLD = 15

# Base autonomous speed
X_SPEED_BASE = 0.03
Y_SPEED_BASE = 0.03

# Rate of base increase per coordinate point off threshold
X_SPEED_RATE = 0.00
Y_SPEED_RATE = 0.00

# Ignore these number of frames if target is missing
LOSS_TOLERANCE = 10

# Attempt to recover for these number of frames if 'missing' frames exceeds tolerance
LOSS_RECOVERY_TOLERANCE = 5

# Frames to ignore before taking action
FRAME_DELAY = 2

MANUAL_SPEED = 1

LAND_SPEED = 1

ALTITUDE_THRESH_HOLD = 500

altitude = 0

RECOVERY_SPEED_X = 0.03
RECOVERY_SPEED_Y = 0.03


def calc_x_speed(x):
    speed = (abs(x) - X_THRESH_HOLD) * (X_SPEED_RATE * X_SPEED_BASE) + X_SPEED_BASE
    return speed if (x < 0) else -1 * speed 

def calc_y_speed(y):
    speed = (abs(y) - Y_THRESH_HOLD) * (Y_SPEED_RATE * Y_SPEED_BASE) + Y_SPEED_BASE
    return speed if (y < 0) else -1 * speed

class DroneDriver(DroneVideoDisplay):

        def __init__(self):
            super(DroneDriver, self).__init__()

            self.pitch = 0
            self.roll = 0
            self.yaw_velocity = 0
            self.z_velocity = 0
 
            self.subTracker = rospy.Subscriber('/rsa_uas/tracker', Tracker, self.ReceiveTracker)

            self.mission_status = 0

            self.loss_count = 0
            self.frame_count = 0

            self.last_x = 0;
            self.last_y = 0;
        
            #self.pubTest = rospy.Publisher('/rsa_uas/tracker', Tracker)
            self.testMsg = Tracker()

            self.testMsg.midpoint.x = 0
            self.testMsg.midpoint.y = 0
            self.testMsg.orientation = 0
            self.testTimer = rospy.Timer(rospy.Duration(100 / 1000.0), self.SendTest)
       
            self.odom_initialised = 0
            self.odom_init = Point32()
            self.odom_curr = Point32()
            self.subOdom = rospy.Subscriber('/ardrone/odometry', Odometry, self.ReceiveOdometry)

	    self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata2)
	    self.altitude = 0

	    self.loss_recovery = False

	def ReceiveNavdata2(self, nav):
	    self.altitude = nav.altd

        def ReceiveOdometry(self, odom):
            if self.odom_initialised == 0:
                self.odom_init = odom.pose.pose.position
                self.odom_initialised = 1

            self.odom_curr = odom.pose.pose.position
            self.odom_curr.x -= self.odom_init.x
            self.odom_curr.y -= self.odom_init.y

            #print("Odom: (" + str(self.odom_curr.x) + ", " + str(self.odom_curr.y)+ ")")

        def SendTest(self, event):
            pass
            #self.pubTest.publish(self.testMsg)
        def ReceiveTracker(self, tracker):
            # position error
            if self.mission_status == 0:
                return
	    if tracker.detection_state != "none":
		print("Altitude: " + str(self.altitude))
		if self.altitude <= ALTITUDE_THRESH_HOLD:
			controller.SendLand()
            #print(tracker)
	        print ("Status: " + tracker.detection_state)
            
            # Increment frame count
            self.frame_count += 1
            
            if tracker.detection_state == "both":
                # Full target was detected
                # Take action if necessary
		self.loss_recovery = False
                if (self.frame_count >= FRAME_DELAY):
                    self.center_midpoint(tracker.midpoint, tracker.orientation)
                    self.frame_count = 0
		else:
		    self.pitch = 0
		    self.roll = 0
                    self.yaw_velocity = 0
                    self.z_velocity = 0
                
	            # UNTESTED
                    '''
                    if tracker.orientation >= 170 or tracker.orientation <= -170 or tracker.orientation >= HEADING_THRESHOLD:
                        self.yaw_velocity = -0.3
                    elif tracker.orientation <= HEADING_THRESHOLD * -1:
                        self.yaw_velocity = 0.3        '''         

		    controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)               
 
                # Reset loss count and last known positions
                self.loss_count = 0
                self.last_x = tracker.midpoint.x
                self.last_y = tracker.midpoint.y
            elif tracker.detection_state == "circle_1":
                # Only circle 1 detected
		self.loss_recovery = False
                if (self.frame_count >= FRAME_DELAY):
                    self.center_midpoint(tracker.midpoint_circle_1, 0)
                    self.frame_count = 0
		else:
		    self.pitch = 0
		    self.roll = 0
                    self.yaw_velocity = 0
                    self.z_velocity = 0
                    controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
                self.loss_count = 0
                self.last_x = tracker.midpoint_circle_1.x
                self.last_y = tracker.midpoint_circle_1.y
            elif tracker.detection_state == "circle_2":
                # Only circle 2 detected
		self.loss_recovery = False
                if (self.frame_count >= FRAME_DELAY):
                    self.center_midpoint(tracker.midpoint_circle_2, 0)
                    self.frame_count = 0
		else:
		    self.pitch = 0
		    self.roll = 0
                    self.yaw_velocity = 0
                    self.z_velocity = 0
                    controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
                self.loss_count = 0
                self.last_x = tracker.midpoint_circle_2.x
                self.last_y = tracker.midpoint_circle_2.y
            else:
                # Nothing detected
                print("Nothing found! =====================================================")

                # Increment loss count
                self.loss_count += 1
                if (self.loss_count > LOSS_TOLERANCE):
                    # Number of losses have exceeded the tolerance, attempt to recover/search
                    
                    if (self.loss_count - LOSS_TOLERANCE > LOSS_RECOVERY_TOLERANCE):
                        # Move the drone towards the last known position
                        p = Point32()
                        p.x = self.last_x
                        p.y = self.last_y
			self.loss_recovery = True
                        self.center_midpoint(p, 0)
                        
                        # Update the last know values so that the drone doesn't keep traveling in the same direction
                        self.last_x += self.roll;
                        self.last_y += self.pitch;

                    else:
                        # Number of losses has exceed the recovery tolerance, resume searching for target
                        # TODO
                        
                        # Rotate around in a circle
                        #self.roll = 0.01
                        #self.yaw_velocity = 0.01
			loss_recovery = False
                        
                        self.pitch = 0
                        self.roll = 0
                        self.yaw_velocity = 0
                        self.z_velocity = 0
			
                        
                        controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

        def center_midpoint(self, point, orientation):
            print(str(point))
            
            # Re-orient the drone if it's stable above the target
            if (point.x > -1 * X_THRESH_HOLD and point.x < X_THRESH_HOLD) and (point.y > -1 * Y_THRESH_HOLD and point.y < Y_THRESH_HOLD):
                '''if orientation >= 170 or orientation <= -170:
                    self.yaw_velocity = -0.3		
                elif orientation >= HEADING_THRESHOLD:
                    self.yaw_velocity = -0.3

                if orientation <= HEADING_THRESHOLD * -1:
                    self.yaw_velocity = 0.3
                else:
                    self.yaw_velocity = 0
	    else:
		self.yaw_velocity = 0'''
		print("In midpoint: " + str(self.mission_status) + " and " + str(self.loss_recovery))
	    	if self.mission_status == 2 and self.altitude > ALTITUDE_THRESH_HOLD and self.loss_recovery == False:
			print("ALTD: " + str(self.altitude) + " ==================Going Down======================= " + str(self.loss_recovery))
			self.z_velocity = -1 * LAND_SPEED
		elif self.altitude <= ALTITUDE_THRESH_HOLD:
			controller.SendLand()

		if orientation >= 170 or orientation <= -170 or orientation >= HEADING_THRESHOLD:
			print("Rotating negative")
                        self.yaw_velocity = -0.1
                elif orientation <= HEADING_THRESHOLD * -1:
			print("Rotating positive")
                        self.yaw_velocity = 0.1                

		    #controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)
			
	    else:
		self.z_velocity = 0
		self.yaw_velocity = 0
            #print ("yaw_velocity = " + str(self.yaw_velocity))
            
            
            # Reposition the drone if necessary
            if point.x >= X_THRESH_HOLD or point.x <= -1 * X_THRESH_HOLD:
                # Drone is not within the x-axis threshold, roll accordingly
		if self.loss_recovery == True:
		    self.roll = RECOVERY_SPEED_X
		    if (point.x > 0):
		        self.roll *= -1
		else:
                	self.roll = calc_x_speed(point.x)
            else:
                # Drone is within the x-axis threshold, zero the roll velocity
                #self.pitch = 0
                self.roll = 0
                #self.yaw_velocity = 0
                #self.z_velocity = 0
            
            if point.y >= Y_THRESH_HOLD or point.y <= -1 * Y_THRESH_HOLD:
                # Drone is not within the y-axis threshold, pitch accordingly
		if self.loss_recovery == True:
			self.pitch = RECOVERY_SPEED_Y
			if (point.y > 0):
				self.pitch *= -1
		else:
        		self.pitch = calc_y_speed(point.y)
            else:
                # Drone is within the y-axis threshold, zero the pitch velocity
                self.pitch = 0
                #self.roll = 0
                #self.yaw_velocity = 0
                #self.z_velocity = 0

	    print ("Pitch = " + str(self.pitch))
	    print ("Roll = " + str(self.roll))
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

        def start_mission(self):
	    print("Odom: (" + str(self.odom_curr.x) + ", " + str(self.odom_curr.y)+ ")")
            controller.SendTakeoff()

        def end_mission(self):
            #controller.SendEmergency()
	    print("Odom: (" + str(self.odom_curr.x) + ", " + str(self.odom_curr.y)+ ")")
            controller.SendLand()
            self.mission_status = 0

        def callBackEnd(self):
            print("Callback end 2")
            self.end_mission()

        def callBackStart(self):
            self.start_mission()

        def callBackTrack(self):
            self.mission_status = 1

        def callBackEmergency(self):
            controller.SendEmergency()

	def callBackLand(self):
	    self.mission_status = 2

	def callBackCamera(self):
	    rospy.wait_for_service('/ardrone/togglecam')
	    try:
		#rospy.ServiceProxy('/ardrone/togglecam', Empty())
		os.system("rosservice call /ardrone/togglecam") #Bit of a hack but works well enough
	    except rospy.ServiceException, e:
		print "Service call failed: %s"%e

        def keyPressEvent(self, event):
            key = event.key()
            print(key)

            # If we have constructed the drone controller and the key is not generated from an auto-repeating key
            if controller is not None and not event.isAutoRepeat():
                # Handle the important cases first!
                if key == KeyMapping.Emergency:
                    print("Emergency")
                    controller.SendEmergency()
                elif key == KeyMapping.Takeoff:
                    #rospy.Publisher('/ardrone/takeoff', Empty).publish(Empty())
                    controller.SendTakeoff()
                elif key == KeyMapping.Land:
                    controller.SendLand()
                else:
                    # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                    if key == KeyMapping.YawLeft:
                        self.yaw_velocity += MANUAL_SPEED
                    elif key == KeyMapping.YawRight:
                        self.yaw_velocity += -1 * MANUAL_SPEED

                    elif key == KeyMapping.PitchForward:
                        self.pitch += MANUAL_SPEED
                    elif key == KeyMapping.PitchBackward:
                        self.pitch += -1 * MANUAL_SPEED

                    elif key == KeyMapping.RollLeft:
                        self.roll += MANUAL_SPEED
                    elif key == KeyMapping.RollRight:
                        self.roll += -1 * MANUAL_SPEED

                    elif key == KeyMapping.IncreaseAltitude:
                        self.z_velocity += 1
                    elif key == KeyMapping.DecreaseAltitude:
                        self.z_velocity += -1

                    elif key == KeyMapping.Up:
                        self.testMsg.midpoint.y -= Y_THRESH_HOLD
                    elif key == KeyMapping.Down:
                        self.testMsg.midpoint.y += Y_THRESH_HOLD

                    elif key == KeyMapping.Right:
                        self.testMsg.midpoint.x += X_THRESH_HOLD
                    elif key == KeyMapping.Left:
                        self.testMsg.midpoint.x -= X_THRESH_HOLD

                    elif key == KeyMapping.Clockwise:
                        self.testMsg.orientation += HEADING_THRESHOLD
                    elif key == KeyMapping.AntiClockwise:
                        self.testMsg.orientation -= HEADING_THRESHOLD

                # finally we set the command to be sent. The controller handles sending this at regular intervals
                controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


        def keyReleaseEvent(self, event):
            key = event.key()

            # If we have constructed the drone controller and the key is not generated from an auto-repeating key
            if controller is not None and not event.isAutoRepeat():
                # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
                # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity -= MANUAL_SPEED
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity -= -1 * MANUAL_SPEED

                elif key == KeyMapping.PitchForward:
                    self.pitch -= MANUAL_SPEED
                elif key == KeyMapping.PitchBackward:
                    self.pitch -= -1 * MANUAL_SPEED

                elif key == KeyMapping.RollLeft:
                    self.roll -= MANUAL_SPEED
                elif key == KeyMapping.RollRight:
                    self.roll -= -1 * MANUAL_SPEED

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity -= MANUAL_SPEED
                elif key == KeyMapping.DecreaseAltitude:
                    self.z_velocity -= -1 * MANUAL_SPEED

                elif key == KeyMapping.Up:
                    #print(self.testMsg.midpoint.y)
                    self.testMsg.midpoint.y += Y_THRESH_HOLD
                    #print(self.testMsg.midpoint.y)
                elif key == KeyMapping.Down:
                    self.testMsg.midpoint.y -= Y_THRESH_HOLD

                elif key == KeyMapping.Right:
                    self.testMsg.midpoint.x -= X_THRESH_HOLD
                elif key == KeyMapping.Left:
                    self.testMsg.midpoint.x += X_THRESH_HOLD

                elif key == KeyMapping.Clockwise:
                    self.testMsg.orientation -= HEADING_THRESHOLD
                elif key == KeyMapping.AntiClockwise:
                    self.testMsg.orientation += HEADING_THRESHOLD



                # finally we set the command to be sent. The controller handles sending this at regular intervals
                controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


if __name__=='__main__':
    rospy.init_node('drone_driver')

    app = QtGui.QApplication(sys.argv)
    controller = Controller()
    display = DroneDriver()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
