#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


def nothing(x):
  pass

class image_converter:

  def __init__(self):
    # Create a black image, a window
    # self.img = np.zeros((300,512,3), np.uint8)

    # cv2.namedWindow("Output Image")
    cv2.startWindowThread()

    cv2.namedWindow('image')

    # create trackbars for color change
    # cv2.createTrackbar('R','image',0,255,nothing)
    # cv2.createTrackbar('G','image',0,255,nothing)
    # cv2.createTrackbar('B','image',0,255,nothing)

    # create switch for ON/OFF functionality
    # self.switch = '0 : OFF \n1 : ON'
    # cv2.createTrackbar(self.switch,'image',0,1,nothing)

    #self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.findXY)
    # cv2.imshow('image',self.img)
    return

  def findXY(self,data):
    rospy.loginfo("Callback findXY called")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    # get current positions of four trackbars
    # r = cv2.getTrackbarPos('R','image')
    # g = cv2.getTrackbarPos('G','image')
    # b = cv2.getTrackbarPos('B','image')
    # s = cv2.getTrackbarPos(self.switch,'image')

    # if s == 0:
    #   img[:] = 0
    # else:
    #   img[:] = [b,g,r]
    # print "r = {0:d} g = {1:d} b = {2:d}".format(r,g,b)
    # print "r = ",r

    # cv2.imshow("Output Image", cv_image)
    # cv2.imshow('image',self.img)
    cv2.waitKey(30)
    # rospy.loginfo("Callback findXY done")
    return


def main(args):
  cv2.namedWindow('image')
  # create trackbars for color change
  cv2.createTrackbar('R','image',0,255,nothing)

  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
