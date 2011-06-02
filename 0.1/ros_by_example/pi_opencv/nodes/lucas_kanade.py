#!/usr/bin/env python

""" lucas_kanade.py - Version 1.0 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys

fc = 0
win_size = 10
MAX_COUNT = 500

pt = None
add_remove_pt = False
flags = 0
night_mode = False
need_to_init = False

class LucasKanadeNode(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name
        
        self.grey = None
        
        """ Set a call back on mouse clicks on the image window """
        cv.SetMouseCallback (self.node_name, self.on_mouse_click, None)
        
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
                        
    def filter_image(self, cv_image):
        global pt, add_remove_pt, night_mode, fc, flags, need_to_init, win_size, MAX_COUNT

        if self.grey is None:
            #self.image = cv.CreateImage (cv.GetSize (cv_image), 8, 3)
            #self.image.origin = cv_image.origin
            self.grey = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage (cv.GetSize (cv_image), 8, 1)
            self.features = []

        # Create a grey version of the image
        cv.CvtColor (cv_image, self.grey, cv.CV_BGR2GRAY)

        if night_mode:
            # Night mode: only display the points
            cv.SetZero (self.image)

        if need_to_init:
            # Create the temporary working images
            eig = cv.CreateImage (cv.GetSize (self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize (self.grey), 32, 1)

            # The default parameters
            quality = 0.01
            min_distance = 10

            # Search for the good features
            self.features = cv.GoodFeaturesToTrack (
                self.grey, eig, temp,
                MAX_COUNT,
                quality, min_distance, None, 3, 0, 0.04)

            # Refine the corner locations
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

        # Processing any keyboard commands
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'c':
                self.features = []
            elif cc == 'n':
                night_mode = not night_mode

def main(args):
    # Display a help message if appropriate.
    help_message =  "Hot keys: \n" \
          "\tESC - quit the program\n" \
          "\tr - auto-initialize tracking\n" \
          "\tc - delete all the points\n" \
          "\tn - switch the \"night\" mode on/off\n" \
          "\tSPACE - next frame\n" \
          "To add/remove a feature point click it\n"
          
    print help_message
    
    # Fire up the Lucas Kanade node.
    LK = LucasKanadeNode("lucas_kanade_tracker")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)