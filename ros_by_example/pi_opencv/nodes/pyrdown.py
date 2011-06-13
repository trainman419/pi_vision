#!/usr/bin/env python

""" pyrdown.py - Version 1.0 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest

NODE_NAME = "pyrdown_filter"

class PyrDownNode(ROS2OpenCV):
    def __init__(self, n):
        ROS2OpenCV.__init__(self, NODE_NAME)
        self.n = n
        
    def filter_image(self, cv_image):
        """ Get the image size """
        image_size = cv.GetSize(cv_image)
        image_width = image_size[0]
        image_height = image_size[1]
        
        n = self.n
        
        scale = 2
        pyramid_images = list()
        pyramid_images.append(cv_image)
        
        for i in range(1, n + 1):  
            pyramid_images.append(cv.CreateImage((image_width/scale, image_height/scale), 8, 3))       
            cv.PyrDown(pyramid_images[i-1], pyramid_images[i])
            #cv.PyrUp(pyramid_images[i], pyramid_images[i-1])
            #cv.Smooth(pyramid_images[i], pyramid_images[i])
            scale = scale * 2

        cv.Resize(pyramid_images[n], cv_image, interpolation=cv.CV_INTER_AREA)

        # Now display the image.
        cv.ShowImage (NODE_NAME, pyramid_images[n])
    
def main(args):
    # Display a help message if appropriate.
    help_message =  ""
          
    print help_message
    
    # Fire up the PyrDown node.
    PD = PyrDownNode(4)

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)