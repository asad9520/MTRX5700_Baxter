#!/usr/bin/env python

import sys
import rospy
import cv2
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import convolutional_neural_network


class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/right_hand_camera/image",Image,self.callback)
    self.dir = os.getcwd() + "/src/facial_recognition/haarcascade_frontalface_default.xml"
  
  def callback(self,data):

    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      
    except CvBridgeError as e:
      print(e)
    
    
    classifier = cv2.CascadeClassifier(self.dir)
    #image = cv2.imread(cv_image)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(gray, cv2.COLOR_RGBA2RGB)
    faceRects = classifier.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=3, minSize=(32, 32))
    
    if len(faceRects) > 0:
        for faceRect in faceRects:
            x, y, w, h = faceRect
            cv2.rectangle(image, (x - 10, y - 10), (x + w + 10, y + h + 10), (0, 255, 0), 2)
            img = gray[(y - 10):(y + h + 10), (x - 10): (x + w + 10)]

            probs, index = convolutional_neural_network.cnn_output(img)
            print(probs)
            if index == 1:
        	    cv2.putText(image, 'Hana', (x-15, y-30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
    cv2.imshow('image window', image)
    cv2.waitKey(3) 

def main(args):
    ic = image_converter()
    rospy.init_node('my_node', anonymous=True)
    try:
      rospy.spin()
       
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
