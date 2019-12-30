#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

roslib.load_manifest('ball_tracker')


bridge = CvBridge()
img_original = np.zeros((480,640,3))
mask = np.zeros((480,640,3))
res = np.zeros((480,640,3))
#frame = np.zeros((480,640,3))
#resg = np.zeros((480,640,3))


def callback(data):

    global bridge
    global img_original
    global mask
    global res
   
    try:
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print e

    # Convert BGR to HSV
    hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([105,100,100])
    upper_blue = np.array([135,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img_original,img_original, mask= mask)
    
     # find contours in the mask to detect object boundary and initialize the current (x, y) center of the ball
    contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    
    # only proceed if at least one contour was found
    if len(contour) > 0:
        # find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
        # The centroid is the weighted average of all the pixels constiturting the shape
        # Compute the largest contour 
        c = max(contour, key = cv2.contourArea)
        # compute the minimum enclosing circle
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        # calculate moments used to compute the center of the image
        M = cv2.moments(c)
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # calculate the center of the object (x, y coordinates)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # draw the contour and center of the shape on the image
            res = cv2.circle(res,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)
            img_original = cv2.circle(img_original,(int(center[0]),int(center[1])),int(radius),(0,255,0),2)
            center_str = Vector3(center[0],center[1],0)
            image_pub=rospy.Publisher('center_position', Vector3, queue_size=10)
            #print center
            image_pub.publish(center_str)

def main():

    global bridge
    global img_original
    global mask
    global res
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera_driver/image_raw', Image, callback)

    while not rospy.is_shutdown():
        cv2.imshow("Converted Image",np.hstack([img_original,res]))
        cv2.waitKey(5) & 0xFF
        rospy.sleep(0.05)

if __name__ == '__main__':
    main()
