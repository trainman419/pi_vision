#!/usr/bin/env python

import roslib
roslib.load_manifest('pi_opencv')
import rospy
import cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from fromROSmsg import read_points
from math import isnan

class DepthViewer:
    def __init__(self):
        rospy.init_node('depth_viewer')

        """ Create the cv_bridge object """
        self.bridge = CvBridge()
        
        self.depth_image = None
        
        cv.NamedWindow("Depth Image", cv.CV_WINDOW_AUTOSIZE)
        cv.ResizeWindow("Depth Image", 640, 480)
        
        self.image_sub = rospy.Subscriber("/camera/depth/image", Image, self.depth_callback)
        #self.cloud_sub = rospy.Subscriber("/camera/rgb/points", PointCloud2, self.cloud_callback)

        
    def depth_callback(self, data):
        depth_image = self.convert_depth_image(data)  
        
        if self.depth_image is None:
            (cols, rows) = cv.GetSize(depth_image)
            self.depth_image = cv.CreateImage((cols, rows), 8, 1)
            rospy.loginfo(cv.GetSize(depth_image))
     
        cv.Copy(depth_image, self.depth_image)
        
        cv.ShowImage("Depth Image", self.depth_image)
        
        """ handle events """
        self.keystroke = cv.WaitKey(6)

        if self.keystroke == 27:
            """ user has press the ESC key, so exit """
            rospy.signal_shutdown("User hit ESC key")
            
    def cloud_callback(self, cloud):
        points = PointCloud2()
        points = read_points(cloud)
        for point in points:
#            p = Point()
#            (p.x, p.y, p.z) = point
            if not isnan(point[0]):
                rospy.loginfo(point)

    def convert_depth_image(self, ros_image):
        try:
            float_depth_image = self.bridge.imgmsg_to_cv(ros_image, "32FC1")
            (cols, rows) = cv.GetSize(float_depth_image)
            depth_image = cv.CreateImage((cols, rows), 8, 1)
            
            """ Convert to an 8-bit image so we can see it """
            for i in range(cols):
                for j in range(rows):
                    depth = cv.Get2D(float_depth_image, j, i)
                    scaled_depth = min(255, int(255 * (max(0, depth[0] - 0.5)) / 5.0))
                    cv.Set2D(depth_image, j, i, scaled_depth)

            return depth_image
    
        except CvBridgeError, e:
            print e
          
def main(args):
    try:
      DepthViewer()
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down viewer."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)