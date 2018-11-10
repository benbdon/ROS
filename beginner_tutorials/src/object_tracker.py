#!/usr/bin/env python

"""
Jarvis Schultz

SUBSCRIBERS:
    - camera_node/image_raw (sensor_msgs/Image)


PUBLISHERS:
    - target_location (geometry_msgs/PointStamped)

SERVICES:

"""

import roslib
import rospy
# import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import tf

import numpy as np

FRAME = "camera_optical_frame"

class ObjectTracker:

    def __init__(self):
        rospy.loginfo("Starting image tracking node")
        # create CV windows for adjusting HSV values:
        cv2.namedWindow("Control", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Control", 1990, 630)
        cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Original", 2034, 30)
        cv2.namedWindow("Thresholded", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Thresholded", 2356, 30)
        cv2.namedWindow("Filled", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Filled", 2679, 30)
        cv2.namedWindow("Contours", cv2.WINDOW_NORMAL)
        # cv2.moveWindow("Contours", 3002, 30)

        # Initialize thresholding values
        low_h  = 0
        high_h = 6
        low_s  = 124
        high_s = 255
        low_v  = 81
        high_v = 218

        # low_h  = 0
        # high_h = 75
        # low_s  = 0
        # high_s = 255
        # low_v  = 214
        # high_v = 255

        # "Do nothing" callback
        def nothing(x):
            pass

        # Create HSV trackbars (nothing on callback)
        cv2.createTrackbar("Low H", "Control", low_h, 179, nothing)
        cv2.createTrackbar("High H", "Control", high_h, 179, nothing)
        cv2.createTrackbar("Low S", "Control", low_s, 255, nothing)
        cv2.createTrackbar("High S", "Control", high_s, 255, nothing)
        cv2.createTrackbar("Low V", "Control", low_v, 255, nothing)
        cv2.createTrackbar("High V", "Control", high_v, 255, nothing)

        # Initialize ROS bridge
        self.bridge = CvBridge()

        # create publisher:
        self.target_pub = rospy.Publisher("target_location", PointStamped,
                                          queue_size=1, latch=True)

        # create camera subscriber:
        self.img_sub = rospy.Subscriber("camera_node/image_raw", Image, self.imagecb)

        return

    def imagecb(self, data):
        """
        Camera Image callback: Converts ROS Image to HSV format,
        thresholds it, performs morphological opening and closing,
        and shows both the original and thresholded images in a
        window. Also publishes necessary information for
        pick_place_controller node
        """
        # Convert Image message to CV image with blue-green-red color order (bgr8)
        try:
            img_original = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print("==[CAMERA MANAGER]==", e)

        # Convert image to HSV format
        img_hsv = cv2.cvtColor(img_original, cv2.COLOR_BGR2HSV)

        # Threshold image based on trackbar values
        low_h  = cv2.getTrackbarPos("Low H", "Control")
        high_h = cv2.getTrackbarPos("High H", "Control")
        low_s  = cv2.getTrackbarPos("Low S", "Control")
        high_s = cv2.getTrackbarPos("High S", "Control")
        low_v  = cv2.getTrackbarPos("Low V", "Control")
        high_v = cv2.getTrackbarPos("High V", "Control")
        img_thresholded = cv2.inRange(img_hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

        # Morphological opening (remove small objects from the foreground)
        img_filled = cv2.erode(img_thresholded, np.ones((4, 4), np.uint8), iterations=2)
        img_filled = cv2.dilate(img_filled, np.ones((4, 4), np.uint8), iterations=2)

        # Morphological closing (fill small holes in the foreground)
        img_filled = cv2.dilate(img_filled, np.ones((6, 6), np.uint8), iterations=1)
        img_filled = cv2.erode(img_filled, np.ones((2, 2), np.uint8), iterations=1)

        # now let's detect the contours in the image:
        img_cont = img_filled.copy()
        contours, _ = cv2.findContours(img_cont, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        # now let's draw the contours
        cv2.drawContours(img_cont, contours, -1, (255,0,0))

        # now choose "most-likely contour"
        xpos = None
        ypos = None
        radius = 0
        for c in contours:
            (x, y), r = cv2.minEnclosingCircle(c)
            if r > 5:
                if r > radius:
                    radius = r
                    xpos = x
                    ypos = y
        if xpos != None and ypos != None:
            cv2.circle(img_cont, (int(xpos), int(ypos)), int(radius), (255,255,255))
            size  = img_hsv.shape
            xpos -= size[1]/2.0
            ypos -= size[0]/2.0
            print "x = ",xpos,"\ty = ",ypos
            # # Publish object location
            pt = PointStamped()
            pt.header.frame_id = FRAME
            pt.header.stamp = data.header.stamp
            pt.point.x = xpos
            pt.point.y = ypos
            self.target_pub.publish(pt)
        else:
            print "no object"
            pass

        # # Calculate the moments of the thresholded image
        # moments = cv2.moments(img_filled)
        # d_m01   = moments["m01"]
        # d_m10   = moments["m10"]
        # d_area  = moments["m00"]

        # # If the area <= 10000, just noise
        # if d_area > 10000:
        #     size  = img_hsv.shape
        #     pos_x = (d_m10 / d_area) - size[1]/2.0
        #     pos_y = (d_m01 / d_area) - size[0]/2.0
        #     print "x = ",pos_x,"\ty = ",pos_y
        #     # # Publish object location
        #     pt = PointStamped()
        #     pt.header.frame_id = FRAME
        #     pt.header.stamp = data.header.stamp
        #     pt.point.x = pos_x
        #     pt.point.y = pos_y
        #     self.target_pub.publish(pt)
        # else:
        #     print "no object"
        #     pass

        # Show the CV image and wait 3ms for a keypress
        cv2.imshow("Original", img_original)
        cv2.imshow("Thresholded", img_thresholded)
        cv2.imshow("Filled", img_filled)
        cv2.imshow("Contours", img_cont)
        cv2.waitKey(10)
        return



def main():
    rospy.init_node('pan_tilt_controller', log_level=rospy.INFO)
    try:
        tracker = ObjectTracker()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__=='__main__':
    main()
