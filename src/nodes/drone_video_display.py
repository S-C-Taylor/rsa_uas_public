#!/usr/bin/env python

# A basic video display window for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This display window listens to the drone's video feeds and updates the display at regular intervals
# It also tracks the drone's status and any connection problems, displaying them in the window's status bar
# By default it includes no control functionality. The class can be extended to implement key or mouse listeners if required

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('rsa_uas')
import rospy

import math

# Import the two types of messages we're interested in
from sensor_msgs.msg import Image    	 # for receiving the video feed
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

# An enumeration of Drone Statuses
from status import Status

#For target tracker info
from rsa_uas.msg import Tracker

# The GUI libraries
from PyQt4.QtCore import *
from PyQt4.QtGui import *


# Some Constants
CONNECTION_CHECK_PERIOD = 250 #ms
GUI_UPDATE_PERIOD = 20 #ms
DETECT_RADIUS = 4 # the radius of the circle drawn when a tag is detected

class FormWidget(QWidget):

    def __init__(self, parent):        
        super(FormWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)

        self.button1 = QPushButton("Start Mission")
        self.layout.addWidget(self.button1)

	self.tempCallBackStart = None
	self.button1.clicked.connect(self.callBackStart)

        self.button2 = QPushButton("End Mission")
        self.layout.addWidget(self.button2)

	self.tempCallBackEnd = None
	self.button2.clicked.connect(self.callBackEnd)

        self.button3 = QPushButton("Start Tracking")
        self.layout.addWidget(self.button3)

	self.tempCallBackTrack = None
	self.button3.clicked.connect(self.callBackTrack)

	self.tempCallBackTrack = None
	self.button3.clicked.connect(self.callBackTrack)

	self.button5 = QPushButton("Start Landing")
        self.layout.addWidget(self.button5)

	self.tempCallBackLand = None
	self.button5.clicked.connect(self.callBackLand)

	self.button4 = QPushButton("Emergency")
        self.layout.addWidget(self.button4)

	self.tempCallBackEmergency = None
	self.button4.clicked.connect(self.callBackEmergency)

	
	self.button6 = QPushButton("Show/Hide Paint")
        self.layout.addWidget(self.button6)

	self.tempCallBackPaint = None
	self.button6.clicked.connect(self.callBackPaint)

	self.button7 = QPushButton("Toggle Camera")
        self.layout.addWidget(self.button7)

	self.tempCallBackCamera = None
	self.button7.clicked.connect(self.callBackCamera)


        self.setLayout(self.layout)

    def callBackStart(self):
	print("callBack Start 1")
	return self.tempCallBackStart()

    def setCallBackStart(self, callback):
	print("setting callback start 1")
	self.tempCallBackStart = callback

    def callBackEnd(self):
	print("callBack End 1")
	return self.tempCallBackEnd()

    def setCallBackEnd(self, callback):
	print("setting callback end 1")
	self.tempCallBackEnd = callback

    def callBackTrack(self):
	print("callBack Track 1")
	return self.tempCallBackTrack()

    def setCallBackTrack(self, callback):
	print("setting callback track 1")
	self.tempCallBackTrack = callback

    def callBackEmergency(self):
	print("callBack Emergency 1")
	return self.tempCallBackEmergency()

    def setCallBackEmergency(self, callback):
	print("setting callback emergency 1")
	self.tempCallBackEmergency = callback

    def callBackLand(self):
	print("callBack Land 1")
	return self.tempCallBackLand()

    def setCallBackLand(self, callback):
	print("setting callback land 1")
	self.tempCallBackLand = callback

    def callBackPaint(self):
	print("callBack Paint 1")
	return self.tempCallBackPaint()

    def setCallBackPaint(self, callback):
	print("setting callback land 1")
	self.tempCallBackPaint = callback

    def callBackCamera(self):
	print("callBack Camera 1")
	return self.tempCallBackCamera()

    def setCallBackCamera(self, callback):
	print("setting callback Camera 1")
	self.tempCallBackCamera = callback


class DroneVideoDisplay(QMainWindow):
	StatusMessages = {
		Status.Emergency : 'Emergency',
		Status.Inited    : 'Initialized',
		Status.Landed    : 'Landed',
		Status.Flying    : 'Flying',
		Status.Hovering  : 'Hovering',
		Status.Test      : 'Test (?)',
		Status.TakingOff : 'Taking Off',
		Status.GotoHover : 'Going to Hover Mode',
		Status.Landing   : 'Landing',
		Status.Looping   : 'Looping (?)'
		}
	DisconnectedMessage = 'Disconnected'
	UnknownMessage = 'Unknown Status'
	
	def __init__(self):
		# Construct the parent class
		super(DroneVideoDisplay, self).__init__()

		# Setup our very basic GUI - a label which fills the whole window and holds our image
		self.setWindowTitle('AR.Drone Video Feed')
		self.imageBox = QLabel(self)
		self.setCentralWidget(self.imageBox)

		bar = self.menuBar()
		file = bar.addMenu("File")
		file.addAction("New")
		file.addAction("save")
		file.addAction("quit")

		self.items = QDockWidget("Dockable", self)
		self.listWidget = QListWidget()
		self.listWidget.addItem("item1")
		self.listWidget.addItem("item2")
		self.listWidget.addItem("item3")
		
		self.form_widget = FormWidget(self)
		self.form_widget.setCallBackStart(self.callBackStart)
		self.form_widget.setCallBackEnd(self.callBackEnd)
		self.form_widget.setCallBackTrack(self.callBackTrack)
		self.form_widget.setCallBackEmergency(self.callBackEmergency)
		self.form_widget.setCallBackLand(self.callBackLand)
		self.form_widget.setCallBackPaint(self.callBackPaint)
		self.form_widget.setCallBackCamera(self.callBackCamera)
		self.items.setWidget(self.form_widget)
		self.items.setFloating(False)
		self.addDockWidget(Qt.RightDockWidgetArea, self.items)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Subscribe to the drone's video feed, calling self.ReceiveImage when a new frame is received
		self.subVideo   = rospy.Subscriber('/ardrone/image_raw',Image,self.ReceiveImage)
		
		# Holds the image frame received from the drone and later processed by the GUI
		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()
		
		# Holds the status message to be displayed on the next GUI update
		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		# A timer to check whether we're still connected
		self.connectionTimer = QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(CONNECTION_CHECK_PERIOD)
		
		# A timer to redraw the GUI
		self.redrawTimer = QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(GUI_UPDATE_PERIOD)

		#Tracker stuff
		self.tracker = Tracker()
		self.tracker_real = Tracker()
		self.subTracker = rospy.Subscriber('/rsa_uas/tracker', Tracker, self.ReceiveTrackerDraw)

		#Paint stuff
		self.show_paint = True
		self.threshold = 150
	
	def ReceiveTrackerDraw(self, tracker):
		self.tracker_real.midpoint.x = tracker.midpoint.x
		self.tracker_real.midpoint.y = tracker.midpoint.y
		self.tracker_real.midpoint_circle_1.x = tracker.midpoint_circle_1.x
		self.tracker_real.midpoint_circle_1.y = tracker.midpoint_circle_1.y
		self.tracker_real.midpoint_circle_2.x = tracker.midpoint_circle_2.x
		self.tracker_real.midpoint_circle_2.y = tracker.midpoint_circle_2.y
		self.tracker_real.camera_image_width = tracker.camera_image_width
		self.tracker_real.camera_image_height = tracker.camera_image_height
		self.tracker_real.detection_state = tracker.detection_state
		#self.tracker = tracker
		
        #Add offsets
		self.tracker.detection_state = tracker.detection_state
		self.tracker.midpoint.x = tracker.midpoint.x + tracker.camera_image_width / 2
		self.tracker.midpoint.y = tracker.midpoint.y + tracker.camera_image_height / 2

		self.tracker.midpoint_circle_1.x = tracker.midpoint_circle_1.x + tracker.camera_image_width / 2
		self.tracker.midpoint_circle_1.y = tracker.midpoint_circle_1.y + tracker.camera_image_height / 2

		self.tracker.midpoint_circle_2.x = tracker.midpoint_circle_2.x + tracker.camera_image_width / 2
		self.tracker.midpoint_circle_2.y = tracker.midpoint_circle_2.y + tracker.camera_image_height / 2

		self.tracker.camera_image_width = tracker.camera_image_width
		self.tracker.camera_image_height = tracker.camera_image_height

	def callBackPaint(self):
		self.show_paint = not self.show_paint

	# Called every CONNECTION_CHECK_PERIOD ms, if we haven't received anything since the last callback, will assume we are having network troubles and display a message in the status bar
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			# We have some issues with locking between the display thread and the ros messaging thread due to the size of the image, so we need to lock the resources

			self.imageLock.acquire()
			try:
				image = QPixmap.fromImage(QImage(self.image.data, self.image.width, self.image.height, QImage.Format_RGB888))
				if self.show_paint:
					painter = QPainter()
					painter.begin(image)
					painter.setPen(QColor(0,255,0))
					painter.setBrush(QColor(0,255,0))
					midpoint = None
				
					if self.tracker.detection_state == "both":
						painter.setPen(QPen(QColor(0,255,0), 10))
						painter.drawLine(self.tracker.midpoint_circle_1.x, self.tracker.midpoint_circle_1.y, self.tracker.midpoint_circle_2.x, self.tracker.midpoint_circle_2.y) #Line between circle 1 and circle 2
						midpoint = self.tracker.midpoint
						rCircle1 = QRect(self.tracker.topleft_circle_1.x, self.tracker.topleft_circle_1.y, 50, 50)
						painter.setPen(QPen(QColor(100,120,126), 5))
						painter.setBrush(QBrush(QColor(0,0,0), 0))
						painter.drawRect(rCircle1)
						pass
					elif self.tracker.detection_state == "circle_1":
						midpoint = self.tracker.midpoint_circle_1
						rCircle1 = QRect(self.tracker.topleft_circle_1.x, self.tracker.topleft_circle_1.y, abs(self.tracker.botright_circle_1.x - self.tracker.topleft_circle_1.x), abs(self.tracker.botright_circle_1.y - self.tracker.topleft_circle_1.y))
						painter.setPen(QPen(QColor(100,120,126), 5))
						painter.setBrush(QBrush(QColor(0,0,0), 0))
						painter.drawRect(rCircle1)
						pass
					elif self.tracker.detection_state == "circle_2":
						midpoint = self.tracker.midpoint_circle_2
						pass

					else:
						pass

					#print(str(midpoint))
					rThresh = QRect(self.tracker.camera_image_width/2 - (self.threshold / 2), self.tracker.camera_image_height/2 - (self.threshold / 2), self.threshold, self.threshold)
					painter.setPen(QPen(QColor(0,0,255), 5))
					painter.setBrush(QBrush(QColor(0,0,0), 0))
					painter.drawRect(rThresh)
					painter.setBrush(QColor(0,0,0))
					painter.setPen(QColor(0,0,0))
					r = QRect(0, 0, 200, 75)
					painter.drawRect(r)

					painter.setBrush(QColor(255,255,255))
					painter.setPen(QPen(QColor(255,255,255), 2))
					rStats = QRectF(5, 5, 195, 50)
					painter.drawText(rStats, QString("Status: " + self.tracker.detection_state))

					rMidpoint = QRectF(5, 25, 195, 50)

					if midpoint == None:
						painter.drawText(rMidpoint, QString("Mid: none"))
					else:
						painter.drawText(rMidpoint, QString("Mid: x = " + str(midpoint.x) + " y = " + str(midpoint.y)))
				

					rOrient = QRectF(5, 45, 195, 50)
					painter.drawText(rOrient, QString("Orientation: " + str(self.tracker.orientation)))


					if midpoint != None:
						painter.setPen(QPen(QColor(255,0,0), 15))
						painter.drawPoint(midpoint.x, midpoint.y)
					
					
					painter.end()

			finally:
				self.imageLock.release()
			'''self.imageLock.acquire()
			
			try:			
					# Convert the ROS image into a QImage which we can display
					image = QPixmap.fromImage(QImage(self.image.data, self.image.width, self.image.height, QImage.Format_RGB888))
					if len(self.tags) > 0:
						self.tagLock.acquire()
						try:
							painter = QPainter()
							painter.begin(image)
							painter.setPen(QColor(0,255,0))
							painter.setBrush(QColor(0,255,0))
							for (x,y,d) in self.tags:
								r = QRectF((x*image.width())/1000-DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,DETECT_RADIUS*2,DETECT_RADIUS*2)
								painter.drawEllipse(r)
								painter.drawText((x*image.width())/1000+DETECT_RADIUS,(y*image.height())/1000-DETECT_RADIUS,str(d/100)[0:4]+'m')
							painter.end()
						finally:
							self.tagLock.release()
			finally:
				self.imageLock.release()

			# We could  do more processing (eg OpenCV) here if we wanted to, but for now lets just display the window.'''
			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		# Update the status bar to show the current drone status & battery level
		self.statusBar().showMessage(self.statusMessage if self.connected else self.DisconnectedMessage)

	def ReceiveImage(self,data):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True

		# We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
		self.imageLock.acquire()
		try:
			self.image = data # Save the ros image for processing by the display thread
		finally:
			self.imageLock.release()

	def ReceiveNavdata(self,navdata):
		# Indicate that new data has been received (thus we are connected)
		self.communicationSinceTimer = True
		
		# Update the message to be displayed
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.UnknownMessage
		self.statusMessage = '{} (Battery: {}%)'.format(msg,int(navdata.batteryPercent))

		self.tagLock.acquire()
		try:
			if navdata.tags_count > 0:
				self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
			else:
				self.tags = []
		finally:
			self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('ardrone_video_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
