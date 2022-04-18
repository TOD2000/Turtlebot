#!/usr/bin/env python

"""
code from
https://github.com/tizianofiorenzani/ros_tutorials/blob/master/opencv/src/find_ball.py
Minor modifications by team turtlebot

ON THE RASPI: roslaunch raspicam_node camerav2_320x240.launch enable_raw:=true
Note - team turtlebot found it usefull to disable the AWB on the camera.  This was added to the launch file.

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic

PUBLISHES TO:
    /balloon/image_balloon : image with detected balooon and search window
    /balloon/image_mask : masking
    /balloon/point_balloon : balloon position in a dimensional values wrt. camera frame

"""



#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.balloon_detector  import *


class BalloonDetector:

    def __init__(self, thr_min, thr_max, blur=15, balloon_params=None, detection_window=None):

        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_balloon_params(balloon_params)
        self.detection_window = detection_window

        self._t0 = time.time()

        self.balloon_point = Point()

        print (">> Publishing image to topic image_balloon")
        self.image_pub = rospy.Publisher('balloon/image_balloon',Image,queue_size=1)
        self.mask_pub = rospy.Publisher('balloon/image_mask',Image,queue_size=1)
        print (">> Publishing position to topic point_balloon")
        self.balloon_pub  = rospy.Publisher('balloon/point_balloon',Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('raspicam_node/image',Image,self.callback)
        print ("<< Subscribed to topic 'raspicam_node/image' ")

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]

    def set_blur(self, blur):
        self._blur = blur

    def set_balloon_params(self, balloon_params):
        self._balloon_params = balloon_params
        


    def callback(self,data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)



        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect balloons
            keypoints, mask   = balloon_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            balloon_params=self._balloon_params, search_window=self.detection_window )
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)

            cv_image    = draw_keypoints(cv_image, keypoints, imshow=True)

            cv2.imshow("Searching Mask", mask)

            #team turtlebot addition added so the display is visible.
            cv2.waitKey(3)

            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)

            for i, keyPoint in enumerate(keypoints):
                #--- Here you can implement some tracking algorithm to filter multiple detections
                #--- We are simply getting the first result


                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))


                #--- Find x and y position in camera adimensional frame
                x, y = get_blob_relative_position(cv_image, keyPoint)
                

                self.balloon_point.x = x
                self.balloon_point.y = y
                self.balloon_pub.publish(self.balloon_point)
                break

            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()

            
def main(args):
    HSV_min = (150,147,150)
    HSV_max = (186,255,255)

    blur     = 5
    min_size = 10
    max_size = 40

    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.05
    x_max   = 0.95
    y_min   = 0.45
    y_max   = 0.85

    detection_window = [x_min, y_min, x_max, y_max]

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 1;
    params.maxThreshold = 100;

    # Filter by Area. - for the balloons this allows good detection to close range.
    params.filterByArea = True
    params.minArea = 300
    params.maxArea = 45000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.2

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.1

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.2

    ic = BalloonDetector(HSV_min, HSV_max, blur, params, detection_window)
    rospy.init_node('balloon_detector', anonymous=True)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)


