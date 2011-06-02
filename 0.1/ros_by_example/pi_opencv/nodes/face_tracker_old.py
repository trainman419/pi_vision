#!/usr/bin/env python

""" face_tracker.py - Version 1.0 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys
import os
from sensor_msgs.msg import RegionOfInterest

class FaceTracker(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name
        
        self.cascade_frontal = rospy.get_param("~cascade_frontal", "")
        self.cascade_profile = rospy.get_param("~cascade_profile", "")
        
        self.cascade_frontal = cv.Load(self.cascade_frontal)
        self.cascade_profile = cv.Load(self.cascade_profile)

        self.min_size = (10, 10)
        self.image_scale = 2
        self.haar_scale = 1.2
        self.min_neighbors = 2
        self.haar_flags = 0
        
        self.tracking = False
        self.grey = None
        self.prev_grey = None
        
        # Set the LK parameters.
        self.night_mode = False       
        self.quality = 0.01
        self.min_distance = 10
        self.win_size = 5
        self.max_count = 200
        self.flags = 0
        
        """ Set up a smaller window to display the CamShift histogram. """
        #cv.NamedWindow("Histogram", 0)
        #cv.MoveWindow("Histogram", 700, 10)
        cv.NamedWindow("Grey", 0)
        cv.MoveWindow("Grey", 700, 10)
        
        self.hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )
        self.backproject_mode = False
        self.init_tracking = True
    
    def filter_image(self, cv_image):
        if self.image_size[0] == 0 or self.image_size[1] == 0:
            return
        
        cv.ShowImage("Grey", self.grey)
        
        if self.tracking:
            self.track_lk(cv_image)
            return
        
        if self.grey is None:
            """ Allocate temporary images """      
            self.grey = cv.CreateImage(self.image_size, 8, 1)
            self.small_image = cv.CreateImage((cv.Round(self.image_size[0] / self.image_scale),
                       cv.Round (self.image_size[1] / self.image_scale)), 8, 1)
    
        """ Convert color input image to grayscale """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        cv.EqualizeHist(self.grey, self.grey)
    
        """ Scale input image for faster processing """
        cv.Resize(self.grey, self.small_image, cv.CV_INTER_LINEAR)
    
        """ Equalize the histogram to reduce lighting effects. """
        cv.EqualizeHist(self.small_image, self.small_image)
    
        """ First check the frontal template """
        if self.cascade_frontal:
            faces = cv.HaarDetectObjects(self.small_image, self.cascade_frontal, cv.CreateMemStorage(0),
                                         self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
            if not faces:
                """ If the frontal template fails, check the profile template """
                if self.cascade_profile:
                    faces = cv.HaarDetectObjects(self.small_image, self.cascade_profile, cv.CreateMemStorage(0),
                                         self.haar_scale, self.min_neighbors, self.haar_flags, self.min_size)
        if not faces:
            return
        
        for ((x, y, w, h), n) in faces:
            """ The input to cv.HaarDetectObjects was resized, so scale the 
                bounding box of each face and convert it to two CvPoints """
            pt1 = (int(x * self.image_scale), int(y * self.image_scale))
            pt2 = (int((x + w) * self.image_scale), int((y + h) * self.image_scale))
            cv.Rectangle(self.image, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
            #if not self.tracking:
            self.track_window = (pt1[0], pt1[1] , pt2[0] - pt1[0], pt2[1] - pt1[1])
            self.tracking = True
            self.init_tracking = True
            self.track_lk(cv_image)
            """ Break out of the loop after the first face """
            return   
        
    def track_lk(self, cv_image):
        if self.init_tracking:
            self.prev_grey = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.prev_pyramid = cv.CreateImage(cv.GetSize (cv_image), 8, 1)
            self.features = []
            self.init_tracking = False
            
        """ Create a grey version of the image """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        cv.EqualizeHist(self.grey, self.grey)
        
        if self.night_mode:
            """ Night mode: only display the points """
            cv.SetZero (self.image)
            
        if self.tracking and self.features != []:
            """ We have feature points, so track and display them """

            """ Calculate the optical flow """
            self.features, status, track_error = cv.CalcOpticalFlowPyrLK(
                self.prev_grey, self.grey, self.prev_pyramid, self.pyramid,
                self.features,
                (self.win_size, self.win_size), 3,
                (cv.CV_TERMCRIT_ITER|cv.CV_TERMCRIT_EPS, 20, 0.03),
                self.flags)
            
#            sum = 0
#            i = 0
#            for error in track_error:
#                sum += error
#                i = i + 1
#            ave_error = sum / i
#            
#            if ave_error > 100:
#                self.tracking = False
#                return
            #rospy.loginfo("Track Error: " + str(ave_error))

            """ Set back the points we keep """
            self.features = [ p for (st,p) in zip(status, self.features) if st]
                            
        elif self.is_rect_nonzero(self.track_window):            
            """ Create the ROI mask"""
            roi = cv.CreateImage(cv.GetSize(cv_image), 8, 1) 
            
            """ Begin with all black pixels """
            cv.Zero(roi)

            """ Get the coordinates and dimensions of the track window """
            x,y,w,h = self.track_window
            
#            x = int(1 * x)
#            y = int(1 * y)
#            w = int(1 * w)
#            h = int(1 * h)
            
            """ Get the center of the track window (type CvRect) so we can create the
                equivalent CvBox2D (rotated rectangle) required by EllipseBox below. """
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)
            roi_box = ((center_x, center_y), (w, h), 0)
            
            """ Create a white ellipse to over the track window to define the ROI.
                A thickness of -1 causes ellipse to be filled with chosen color,
                in this case white. """
            cv.EllipseBox(roi, roi_box, cv.CV_RGB(255,255, 255), thickness=-1)
            
            """ Create the temporary scratch pad images """
            eig = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
            temp = cv.CreateImage (cv.GetSize(self.grey), 32, 1)
            
#            (features, descriptors) = cv.ExtractSURF(self.grey, roi, cv.CreateMemStorage(0), (0, 500, 3, 1))
#            for feature in features:
#                self.features.append(feature[0])

            """ Search for the good points to track """
            self.features = cv.GoodFeaturesToTrack(
                self.grey, eig, temp,
                self.max_count,
                self.quality, self.min_distance, roi, 3, 0, 0.04)

            """ Refine the corner locations """
            self.features = cv.FindCornerSubPix(
                self.grey,
                self.features,
                (self.win_size, self.win_size),  (-1, -1),
                (cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 20, 0.03))
            
            self.tracking = True    

        """ Swapping the images """
        self.prev_grey, self.grey = self.grey, self.prev_grey
        self.prev_pyramid, self.pyramid = self.pyramid, self.prev_pyramid
        
        """ The FitEllipse2 function below requires us to convert the feature array
            into a CvMat matrix """
        try:
            self.feature_matrix = cv.CreateMat(1, len(self.features), cv.CV_32SC2)
        except:
            pass
                    
        """ Draw the points as green circles and add them to the features matrix """
        i = 0
        for the_point in self.features:
            cv.Circle(self.image, (int(the_point[0]), int(the_point[1])), 3, (0, 255, 0, 0), -1, 8, 0)
            try:
                cv.Set2D(self.feature_matrix, 0, i, (int(the_point[0]), int(the_point[1])))
            except:
                pass
            i = i + 1

        """ Draw the best fit ellipse around the feature points """
        if len(self.features) > 6:
            face_box = cv.FitEllipse2(self.feature_matrix)
            cv.EllipseBox(self.image, face_box, cv.CV_RGB(255,255, 0), 1)

        """ Process any keyboard commands """
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'c':
                self.features = []
            elif cc == 'n':
                self.night_mode = not self.night_mode
        
        """ Publish the ROI for the tracked face """
        try:
            (roi_center, roi_size, roi_angle) = face_box
            if not self.drag_start and self.is_rect_nonzero(self.track_window):     
                self.ROI = RegionOfInterest()
                self.ROI.x_offset = roi_center[0] - roi_size[0] / 2
                self.ROI.y_offset = roi_center[1] - roi_size[1] / 2
                self.ROI.width = roi_size[0]
                self.ROI.height = roi_size[1]
                
            self.pubROI.publish(self.ROI)
        except:
            pass
        

    def track_camshift(self, cv_image):
        if not self.tracking:
            """ Get the image size """
            image_size = cv.GetSize(cv_image)
            image_width = image_size[0]
            image_height = image_size[1]
            
            """ Convert to HSV and keep the hue """
            hsv = cv.CreateImage(image_size, 8, 3)
            cv.CvtColor(cv_image, hsv, cv.CV_BGR2HSV)
            self.hue = cv.CreateImage(image_size, 8, 1)
            cv.Split(hsv, self.hue, None, None, None)
    
            """ Compute back projection """
            backproject = cv.CreateImage(image_size, 8, 1)
            
            """ Get the initial histogram """
            sel = cv.GetSubRect(self.hue, self.track_window )
            cv.CalcArrHist( [sel], self.hist, 0)
            (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
            if max_val != 0:
                cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
            self.tracking_face = True

        """ Run the CamShift algorithm """
        cv.CalcArrBackProject( [self.hue], backproject, self.hist )
        if self.track_window and self.is_rect_nonzero(self.track_window):
            crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
            (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.track_window, crit)
            self.track_window = rect
            cv.EllipseBox(cv_image, track_box, cv.CV_RGB(255,255,255), 3, cv.CV_AA, 0)
        else:
            self.tracking_face = False

        cv.ShowImage("Histogram", self.hue_histogram_as_image(self.hist))
        
    def hue_histogram_as_image(self, hist):
            """ Returns a nice representation of a hue histogram """
    
            histimg_hsv = cv.CreateImage( (320,200), 8, 3)
            
            mybins = cv.CloneMatND(hist.bins)
            cv.Log(mybins, mybins)
            (_, hi, _, _) = cv.MinMaxLoc(mybins)
            cv.ConvertScale(mybins, mybins, 255. / hi)
    
            w,h = cv.GetSize(histimg_hsv)
            hdims = cv.GetDims(mybins)[0]
            for x in range(w):
                xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
                val = int(mybins[int(hdims * x / w)] * h / 255)
                cv.Rectangle( histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
                cv.Rectangle( histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)
    
            histimg = cv.CreateImage( (320,200), 8, 3)
            cv.CvtColor(histimg_hsv, histimg, cv.CV_HSV2BGR)
            return histimg   
    
def main(args):
    """ Display a help message if appropriate """
    help_message =  ""
          
    print help_message
    
    """ Fire up the Face Tracker node """
    FT = FaceTracker("face_tracker")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)