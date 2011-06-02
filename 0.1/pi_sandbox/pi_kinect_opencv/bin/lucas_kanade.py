#!/usr/bin/env python

""" lucas_kanade.py - Version 1.0 2011-04-19

    Modification of the ROS OpenCV lkdemo example using cv_bridge and publishing the ROI
    coordinates to the /roi topic.   
"""

import roslib
roslib.load_manifest('pi_kinect_opencv')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os

fc = 0
win_size = 10
MAX_COUNT = 500


pt = None
add_remove_pt = False
flags = 0
night_mode = False
need_to_init = False

class LucasKanadeNode:
    def __init__(self):
        rospy.init_node('lucas_kanade_node')
           
        self.ROI = rospy.Publisher("roi", RegionOfInterest)
        
        self.image = None

        """ Create the display window """
        self.cv_window_name = "LKTracker"
        cv.NamedWindow(self.cv_window_name, 0)
        
        """ Create the cv_bridge object """
        self.bridge = CvBridge()
        
        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        
        rospy.wait_for_message('/camera/rgb/image_color', Image)
        
        cv.SetMouseCallback ('LKTracker', self.on_mouse_click, None)

    def on_mouse_click(self, event, x, y, flags, param):   
        # we will use the global pt and add_remove_pt
        global pt, add_remove_pt
        
        if self.image is None:
            # not initialized, so skip
            return
    
        if self.image.origin != 0:
            # different origin
            y = self.image.height - y
    
        if event == cv.CV_EVENT_LBUTTONDOWN:
            # user has click, so memorize it
            pt = (x, y)
            add_remove_pt = True

    def image_callback(self, data):
        """ Convert the raw image to OpenCV format using the convert_image() helper function """
        cv_image = self.convert_image(data)
        
        """ Apply the CamShift algorithm using the do_camshift() helper function """
        cv_image = self.do_lucas_kanade(cv_image)
        
        """ Refresh the displayed image """
        cv.ShowImage(self.cv_window_name, cv_image)
        
        # handle events
        c = cv.WaitKey(7) % 0x100

        if c == 27:
            # user has press the ESC key, so exit
            return

        # processing depending on the character
        if 32 <= c and c < 128:
          cc = chr(c).lower()
          if cc == 'r':
              need_to_init = True
          elif cc == 'c':
              self.features = []
          elif cc == 'n':
              self.night_mode = not night_mode
          elif cc == ' ':
              self.fc = (self.fc + 1) % len(frames)
          
    def convert_image(self, ros_image):
        try:
          cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
          return cv_image
        except CvBridgeError, e:
          print e
          
    def do_lucas_kanade(self, cv_image):
        global pt, add_remove_pt, night_mode, fc, flags, need_to_init, win_size, MAX_COUNT

        if self.image is None:
            rospy.loginfo("HELLO!")
            self.image = cv.CreateImage (cv.GetSize (cv_image), 8, 3)
            #self.image.origin = cv_image.origin
            self.grey = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.features = []
            #os._exit(1)

        # copy the frame, so we can draw on it
        cv.Copy (cv_image, self.image)

        # create a grey version of the image
        cv.CvtColor (self.image, self.grey, cv.CV_BGR2GRAY)

        if night_mode:
            # night mode: only display the points
            cv.SetZero (self.image)

        if need_to_init:
            # we want to search all the good points

            # create the wanted images
            eig = cv.CreateImage (cv.GetSize (self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize (self.grey), 32, 1)

            # the default parameters
            quality = 0.01
            min_distance = 10

            # search the good points
            self.features = cv.GoodFeaturesToTrack (
                self.grey, eig, temp,
                MAX_COUNT,
                quality, min_distance, None, 3, 0, 0.04)

            # refine the corner locations
            self.features = cv.FindCornerSubPix (
                self.grey,
                self.features,
                (win_size, win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.03))

        elif self.features != []:
            # we have points, so display them

            # calculate the optical flow
            self.features, status, track_error = cv.CalcOpticalFlowPyrLK (
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                self.features,
                (win_size, win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
                flags)

            # set back the points we keep
            self.features = [ p for (st,p) in zip(status, self.features) if st]

            if add_remove_pt:
                # we have a point to add, so see if it is close to
                # another one. If yes, don't use it
                def ptptdist(p0, p1):
                    dx = p0[0] - p1[0]
                    dy = p0[1] - p1[1]
                    return dx**2 + dy**2
                if min([ ptptdist(pt, p) for p in self.features ]) < 25:
                    # too close
                    add_remove_pt = 0

            # draw the points as green circles
            for the_point in self.features:
                cv.Circle (self.image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), -1, 8, 0)
            
        if add_remove_pt:
            # we want to add a point
            # refine this corner location and append it to 'features'

            self.features += cv.FindCornerSubPix (
                self.grey,
                [pt],
                (win_size, win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS,
                20, 0.03))
            # we are no longer in "add_remove_pt" mode
            add_remove_pt = False

        # swapping
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        need_to_init = False
        
        # we can now display the image
        cv.ShowImage ('LKTracker', self.image)


def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)           

def main(args):
    # Display a small howto use it
    print "Hot keys: \n" \
          "\tESC - quit the program\n" \
          "\tr - auto-initialize tracking\n" \
          "\tc - delete all the points\n" \
          "\tn - switch the \"night\" mode on/off\n" \
          "\tSPACE - next frame\n" \
          "To add/remove a feature point click it\n"
          
    # Fire up the node.
    lk = LucasKanadeNode()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
