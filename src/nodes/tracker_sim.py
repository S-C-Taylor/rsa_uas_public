#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import message_filters
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rsa_uas.msg import Tracker

pubTracker = 0

def processImage(color):
    bridge = CvBridge()
    # convert rosImg to openCv image
    try:
        colorImage = bridge.imgmsg_to_cv2(color, "rgb8")
    except CvBridgeError, e:
        print e

    height, width, c = colorImage.shape

    # convert image to a hsv image
    hsvImage = cv2.cvtColor(colorImage, cv2.COLOR_BGR2HSV)

    # boundries for colours in HSV format (still need blue and green)

    # Pink-Blue no board
    #blueLower = np.array([10, 150, 125])
    #blueUpper = np.array([25, 220, 255])

    # Pink-Yellow no board
    #blueLower = np.array([85, 70, 200])
    #blueUpper = np.array([100, 255, 255])

    # Simulator
    blueLower = np.array([36, 108, 117])
    blueUpper = np.array([120, 168, 177])

    # Pink-Green on board
    #blueLower = np.array([35, 105, 65])
    #blueUpper = np.array([80, 225, 125])

    # Simulator/Pink-Blue no board
    pinkLower = np.array([125, 70, 140])
    pinkUpper = np.array([165, 215, 255])

    # Pink-Green on board
    #pinkLower = np.array([125, 150, 85])
    #pinkUpper = np.array([165, 255, 200])

    # obtain a mask for each of the colour
    maskBlue = cv2.inRange(hsvImage, blueLower, blueUpper)
    maskPink = cv2.inRange(hsvImage, pinkLower, pinkUpper)

    # remove noise from each of the masks
    kernel = np.ones((5, 5), np.uint8)
    #maskBlue = cv2.erode(maskBlue, kernel, iterations=1)
    #maskPink = cv2.erode(maskPink, kernel, iterations=1)

    # count the number of each colour
    numBlue = cv2.countNonZero(maskBlue)
    numPink = cv2.countNonZero(maskPink)

    #cv2.imshow("tracking", colorImage)
    #cv2.waitKey(1)

    contours, _ = cv2.findContours(maskPink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pinkFound = (len(contours) != 0)

    midPink = Point32()
    topLeftPink = Point32()
    botRightPink = Point32()
    if (pinkFound):
        cnt = contours[0]
        for c in contours:
            if cv2.contourArea(cnt) < cv2.contourArea(c):
                cnt = c

        if cv2.contourArea(cnt) < 10:
            pinkFound = False

        x, y, w, h = cv2.boundingRect(cnt)
 
        topLeftPink.x = x - width / 2
        topLeftPink.y = y - height / 2

        topLeftPink.z = 0
        botRightPink.x = x + w - width / 2
        botRightPink.y = y + h - height / 2
        botRightPink.z = 0
        p1 = (x, y)
        p2 = (x + w, y + h)
        cv2.rectangle(colorImage, p1, p2, (0,0,0))


        midPink.x = (topLeftPink.x + botRightPink.x) / 2
        midPink.y = (topLeftPink.y + botRightPink.y)/ 2
        midPink.z = 0

    contours, _ = cv2.findContours(maskBlue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blueFound = (len(contours) != 0)

    midBlue = Point32()
    topLeftBlue = Point32()
    botRightBlue = Point32()
    if (blueFound):
        cnt = contours[0]
        for c in contours:
            if cv2.contourArea(cnt) < cv2.contourArea(c):
                cnt = c

            if cv2.contourArea(cnt) < 10:
                blueFound = False


        x, y, w, h = cv2.boundingRect(cnt)

        topLeftBlue.x = x - width / 2
        topLeftBlue.y = y - height / 2
        topLeftBlue.z = 0
        botRightBlue.x = x + w - width / 2
        botRightBlue.y = y + h - height / 2
        botRightBlue.z = 0
        
	p1 = (x, y)
	p2 = (x + w, y + h)
        cv2.rectangle(colorImage, p1, p2, (0,0,0))

        midBlue.x = (topLeftBlue.x + botRightBlue.x) / 2
        midBlue.y = (topLeftBlue.y + botRightBlue.y) / 2
        midBlue.z = 0

    midPoint = Point32()
    midPoint.x = (midPink.x + midBlue.x) / 2
    midPoint.y = (midPink.y + midBlue.y) / 2
    midPoint.z = 0

    retData = Tracker()
    retData.orientation = 0
    retData.camera_image_width = width
    retData.camera_image_height = height
    retData.detection_state = "both"
    retData.midpoint = midPoint
    retData.midpoint_circle_1 = midPink
    retData.topleft_circle_1 = topLeftPink
    retData.botright_circle_1 = botRightPink
    retData.midpoint_circle_2 = midBlue
    retData.topleft_circle_2 = topLeftBlue
    retData.botright_circle_2 = botRightBlue

    global pubTracker

    if (not blueFound and not pinkFound):
        retData.detection_state = "none"
        pubTracker.publish(retData)
        return
    elif (not blueFound and pinkFound):
        retData.detection_state = "circle_1"
        pubTracker.publish(retData)
        return
    elif (blueFound and not pinkFound):
        retData.detection_state = "circle_2"
        pubTracker.publish(retData)
        return

    opp = abs(midPink.y - midBlue.y)
    adj = abs(midPink.x - midBlue.x)

    alpha = 0
    if (adj != 0) :
        alpha = math.degrees(math.atan(opp / adj))

        if (midPink.x == midBlue.x):
            # Y-axis aligned
            if (midPink.y < midBlue.y):
                alpha = 0
            elif (midPink.y > midBlue.y):
                alpha = 180
            else:
                return
        elif (midPink.y == midBlue.y):
            # X-axis aligned
            if (midPink.x > midBlue.x):
                alpha = 90
                if (midPink.x < midBlue.x):
                    alpha = -90
        elif (midPink.x < midBlue.x and midPink.y > midBlue.y):
            # Quadrant 1
            alpha = -90 - alpha
        elif (midPink.x > midBlue.x and midPink.y > midBlue.y):
            # Quadrant 2
            alpha += 90
        elif (midPink.x > midBlue.x and midPink.y < midBlue.y):
            # Quadrant 3
            alpha = 90 - alpha
        elif (midPink.x < midBlue.x and midPink.y < midBlue.y):
            # Quadrant 4
            alpha -= 90

    cv2.imshow("tracking", colorImage)
    cv2.waitKey(1)

    retData.orientation = alpha
    pubTracker.publish(retData)

def listener():
    rospy.init_node('target_tracker')
    rospy.Subscriber("/ardrone/image_raw", Image, processImage)
    rospy.spin()


if __name__ == '__main__':
    try:
        pubTracker = rospy.Publisher("/rsa_uas/tracker", Tracker, queue_size = 10)
        listener()
    except rospy.ROSInterruptException:
        pass
