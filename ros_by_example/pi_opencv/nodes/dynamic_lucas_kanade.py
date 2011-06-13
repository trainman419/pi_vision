#!/usr/bin/env python

""" lucas_kanade.py - Version 1.0 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys

class LucasKanadeNode(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name
        self.grey = None

        # Set the LK parameters.
        self.night_mode = False       
        self.quality = 0.01
        self.min_distance = 10
        self.win_size = 10
        self.max_count = 500
        self.flags = 0
                        
    def filter_image(self, cv_image):
        if self.grey is None:
            self.grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.features = []

        # Create a grey version of the image
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)

        if self.night_mode:
            # Night mode: only display the points
            cv.SetZero (self.image)
            
        if self.features != []:
            # We have feature points, so display them

            # Calculate the optical flow
            self.features, status, track_error = cv.CalcOpticalFlowPyrLK(
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                self.features,
                (self.win_size, self.win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
                self.flags)

            # Set back the points we keep
            self.features = [ p for (st,p) in zip(status, self.features) if st]        

            # Draw the points as green circles
            for the_point in self.features:
                cv.Circle(self.image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), -1, 8, 0)

            
        """ If mouse is pressed, recompute the features to track """
        if self.drag_start and self.is_rect_nonzero(self.selection):
            sub = cv.GetSubRect(cv_image, self.selection)
            save = cv.CloneMat(sub)
            cv.ConvertScale(cv_image, cv_image, 0.5)
            cv.Copy(save, sub)
            x,y,w,h = self.selection
           
            # Create the ROI mask 
            roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            # Begin with all black.
            cv.Zero(roi)
            
            # Create a white rectangle over the track window. A thickness of -1 causes rectangle to be filled with chosen color, in this case white.
            cv.Rectangle(roi, (x, y), (x + w, y + h), cv.CV_RGB(255,255, 255), -1, 8, 0)
            
            # Create the temporary images
            eig = cv.CreateImage(cv.GetSize (self.grey), 32, 1)
            temp = cv.CreateImage(cv.GetSize (self.grey), 32, 1)

            # Search the good points
            self.features = cv.GoodFeaturesToTrack(
                self.grey, eig, temp,
                self.max_count,
                self.quality, self.min_distance, roi, 3, 0, 0.04)

            # Refine the corner locations
            self.features = cv.FindCornerSubPix(
                self.grey,
                self.features,
                (self.win_size, self.win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.03))
            
        elif self.track_window and self.is_rect_nonzero(self.track_window):
            pass

        # Swapping
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        # Processing any keyboard commands
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'c':
                self.features = []
            elif cc == 'n':
                self.night_mode = not self.night_mode


def main(args):
    # Display a help message if appropriate.
    help_message =  "Hot keys: \n" \
          "\tESC - quit the program\n" \
          "\tc - delete all the points\n" \
          "\tn - switch the \"night\" mode on/off\n"
          
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