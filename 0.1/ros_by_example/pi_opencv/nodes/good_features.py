#!/usr/bin/env python

""" good_features.py - Version 1.0 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest

NODE_NAME = "good_features_to_track"

class GoodFeaturesNode(ROS2OpenCV):
    def __init__(self):
        ROS2OpenCV.__init__(self, NODE_NAME)

        
    def filter_image(self, cv_image):
        """ Get the image size """
        image_size = cv.GetSize(cv_image)
        image_width = image_size[0]
        image_height = image_size[1]
       
        roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
        grey_image = cv.CreateImage(cv.GetSize(cv_image), 8, 1)
        cv.CvtColor(cv_image, grey_image, cv.CV_RGB2GRAY) 
        eig_image = cv.CreateImage(cv.GetSize(grey_image), cv.IPL_DEPTH_32F, 1)
        temp_image = cv.CreateImage(cv.GetSize(grey_image), cv.IPL_DEPTH_32F, 1)

        cv.Zero(roi)

        cv.Circle(roi, (200,200), 50, cv.CV_RGB(255,255, 255), -1, 8, 0)

        pts = cv.GoodFeaturesToTrack(grey_image, eig_image, temp_image, 100, 0.09, 2, roi, useHarris=1)
        for point in pts:
            cv.Line(image, point, point, cv.RGB(0,255,255), 3)
            
     
        """ If mouse is pressed, highlight the current selected rectangle
            and recompute the histogram """
        if self.drag_start and self.is_rect_nonzero(self.selection):
            sub = cv.GetSubRect(cv_image, self.selection)
            save = cv.CloneMat(sub)
            cv.ConvertScale(cv_image, cv_image, 0.5)
            cv.Copy(save, sub)
            x,y,w,h = self.selection
            cv.Rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 255))

            sel = cv.GetSubRect(self.hue, self.selection )
            cv.CalcArrHist( [sel], self.hist, 0)
            (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
            if max_val != 0:
                cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
        elif self.track_window and self.is_rect_nonzero(self.track_window):
            #cv.Rectangle(cv_image, (int(track_box[0][0] - track_box[1][0] / 2), int(track_box[0][1] - track_box[1][1] / 2)), (int(track_box[0][0] + track_box[1][0] / 2), int(track_box[0][1] + track_box[1][1] / 2)), (0, 255, 255))
            cv.EllipseBox( cv_image, track_box, cv.CV_RGB(255,0,0), 3, cv.CV_AA, 0 )
            
            self.ROI = RegionOfInterest()
            self.ROI.x_offset = int(min(image_width, max(0, track_box[0][0] - track_box[1][0] / 2)))
            self.ROI.y_offset = int(min(image_height, max(0, track_box[0][1] - track_box[1][1] / 2)))
            self.ROI.width = int(track_box[1][0])
            self.ROI.height = int(track_box[1][1])
                    
            self.pubROI.publish(self.ROI)
            
#            roi = RegionOfInterest()
#            roi.x_offset = int(min(image_width, max(0, track_box[0][0] - track_box[1][0] / 2)))
#            roi.y_offset = int(min(image_height, max(0, track_box[0][1] - track_box[1][1] / 2)))
#            roi.width = int(track_box[1][0])
#            roi.height = int(track_box[1][1])
#            self.ROI.publish(roi)

        # Now display the image.
        cv.ShowImage (NODE_NAME, cv_image)

        cv.ShowImage("Histogram", self.hue_histogram_as_image(self.hist))
        
        if not self.backproject_mode:
            return cv_image
        else:
            return backproject
            
#void updateHueImage(const IplImage * pImg)
#{
#    // Convert to HSV color model
#    cvCvtColor( pImg, pHSVImg, CV_BGR2HSV );
#
#    // Mask out-of-range values
#    cvInRangeS( pHSVImg, cvScalar(0, smin, MIN(vmin,vmax), 0),
#                cvScalar(180, 256, MAX(vmin,vmax) ,0), pMask );
#
#    // Extract the hue channel
#    cvSplit( pHSVImg, pHueImg, 0, 0, 0 );
#}


        
def main(args):
    # Display a help message if appropriate.
    help_message =  ""
          
    print help_message
    
    # Fire up the Good Features node.
    GF = GoodFeaturesNode()

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)