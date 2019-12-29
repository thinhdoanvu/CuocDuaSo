#!/usr/bin/env python2
"""OpenCV feature detectors with ros CompressedImage Topics in python.
This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
from std_msgs.msg import Int32
# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError
 
VERBOSE=False 

ketQua = 0

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("HTH/camera/rgb/compressed", CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print("subscribed to /camera/image/compressed")

    def callback(self, ros_data):
        
        global ketQua

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
   
        #chuyen sang HSV
        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)

        #nguong mau xanh theo HSV
        lower_color = np.array([80,70,20])#70/50/20
        upper_color = np.array([125,255,255])#125/255/255

        #chuyen sang anh nhi phan theo nguong mau
        mask = cv2.inRange(hsv, lower_color, upper_color)
        mask2 = cv2.inRange(hsv, lower_color, upper_color)
        #ma tran gian no anh
        kernel = np.ones((2,2))
        kernel2 = np.ones((5,5))
        #khu nhieu theo kernel
        mask = cv2.erode(mask, kernel)
        #gian no anh theo kernel2
        mask = cv2.dilate(mask, kernel2, iterations=2)

        #ham tim vat the
        _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)

        countContours = 0
        for contour in contours:
            x, y, w, h = contourRect = cv2.boundingRect(contour)
            
            if 300< w * h < 10000:
                countContours += 1
                x2 = x + w
                y2 = y + h
                a = x2 - x
                b = y2 - y
                dienTich = a*b
                # print(dienTich)
                if a % b < 50 and y > 40 and x2 < 320 and x2 > 300 and dienTich > 750 and dienTich < 4000:
                    #ve hinh vuong theo toa do
                    cv2.rectangle(image_np, (x, y), (x2, y2), (0, 255, 0))
                    x_bb = x
                    y_bb = y
                    x2_bb = x2
                    y2_bb = y2
                    break
       
        try:
            #cat lay vat the
            cropped = mask2[y_bb:y2_bb, x_bb:x2_bb]
            xtb = (x_bb + x2_bb)/2
            ytb = (y_bb + y2_bb)/2
            #chia vat the thanh 2 phan
            cropped1 = mask2[y_bb:ytb, x_bb:xtb]
            cropped2 = mask2[y_bb:ytb, xtb:x2_bb]
            cropped3 = mask2[ytb:y2_bb, x_bb:xtb]
            cropped4 = mask2[ytb:y2_bb, xtb:x2_bb]

            #dem pixel 
            C1 = np.count_nonzero(cropped1)
            C2 = np.count_nonzero(cropped2)
            C3 = np.count_nonzero(cropped3)
            C4 = np.count_nonzero(cropped4)

            pixel_trai = C3/C1
            pixel_phai = C4/C2

            if pixel_trai > pixel_phai:
                ketQua = 1 #re trai
                #print('re trai')
            else:
                ketQua = 2 #re phai
                #print('re phai')

            # cv2.imshow('cropped', cropped)
            # cv2.imshow('c1', cropped1)
            # cv2.imshow('c2', cropped2)
        except:
            pass

        #cv2.imshow('mask', mask)
        #cv2.imshow('mask2', mask2)
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    global ketQua
    rospy.init_node('image_feature', anonymous=True)
    rate = rospy.Rate(1)
    pub = rospy.Publisher('bienBao', Int32, queue_size=10)
    while not rospy.is_shutdown():
        #rospy.loginfo(ketQua)
        pub.publish(ketQua)
        if ketQua == 1 or ketQua == 2:
            time.sleep(4)
            ketQua = 0
        rate.sleep()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
