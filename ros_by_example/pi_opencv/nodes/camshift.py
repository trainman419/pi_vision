#!/usr/bin/env python

""" camshift.py - Version 0.1 2011-04-28
      
"""

import roslib
roslib.load_manifest('pi_opencv')
from ros2opencv import ROS2OpenCV
import rospy
import cv
import sys
from sensor_msgs.msg import RegionOfInterest
import numpy

class CamShiftNode(ROS2OpenCV):
    def __init__(self, node_name):
        ROS2OpenCV.__init__(self, node_name)
        
        self.node_name = node_name
        
        """ Set up a smaller window to display the CamShift histogram. """
        cv.NamedWindow("Histogram", 0)
        cv.MoveWindow("Histogram", 700, 10)
        cv.NamedWindow("Mask", 0)
        cv.MoveWindow("Mask", 700, 500)

        self.hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )
        self.backproject = None
        self.backproject_mode = False
        self.hsv = None
        self.hue = None
        
        """ Set saturation and value thresholds for the hue image """
        self.vmin = 79
        self.vmax = 200
        self.smin = 29
        
    def process_image(self, cv_image):
        
        if self.detect_box:
            if not self.track_box or not self.is_rect_nonzero(self.track_box):
                self.track_box = self.detect_box
            else:
                self.track_box = self.camshift(cv_image)
        
        elif self.drag_start and self.is_rect_nonzero(self.selection):
            self.show_histogram(cv_image)
        
        return cv_image
    
    def show_histogram(self, cv_image):
        if not self.hsv:
            self.hsv = cv.CreateImage(self.image_size, 8, 3)
            self.hue = cv.CreateImage(self.image_size, 8, 1)
            
        cv.CvtColor(cv_image, self.hsv, cv.CV_BGR2HSV)
        cv.Split(self.hsv, self.hue, None, None, None)
            
        sub = cv.GetSubRect(cv_image, self.selection)
        save = cv.CloneMat(sub)
        cv.ConvertScale(cv_image, cv_image, 0.5)
        cv.Copy(save, sub)

        sel = cv.GetSubRect(self.hue, self.selection )
        cv.CalcArrHist([sel], self.hist, 0)
        (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
        if max_val != 0:
            cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
        cv.ShowImage( "Histogram", self.hue_histogram_as_image(self.hist))

    
    def camshift(self, cv_image):
        if not self.backproject:
            self.backproject = cv.CreateImage(self.image_size, 8, 1)
            self.backproject_rgb = cv.CreateImage(self.image_size, 8, 3)
            self.track = cv.CreateImage(self.image_size, 8, 3)
            self.hsv = cv.CreateImage(self.image_size, 8, 3)
            self.hue = cv.CreateImage(self.image_size, 8, 1)
            self.mask = cv.CreateImage(self.image_size, 8, 1)
                    
        """ Convert to HSV and keep the hue """
        cv.CvtColor(cv_image, self.hsv, cv.CV_BGR2HSV)
        
        #cv.InRangeS(self.hsv, (120, self.smin, min(self.vmin, self.vmax), 0), (180, 256, max(self.vmin, self.vmax) ,0), self.mask)
        cv.Split(self.hsv, self.hue, None, None, None)
        
        """ Run the CamShift algorithm """
        cv.CalcArrBackProject([self.hue], self.backproject, self.hist)
                
        (_, max_val, _, _) = cv.GetMinMaxHistValue(self.hist)
        if max_val != 0:
            cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
            
        #cv.CalcBackProject(self.hsv, self.backproject, self.hist)
        #cv.And(self.backproject, self.mask, self.backproject, None)
        
        if self.track_box and self.is_rect_nonzero(self.track_box):
            crit = (cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
            (iters, (area, value, rect), camshift_box) = cv.CamShift(self.backproject, self.track_box, crit)
            #self.track_box = rect       
     
                
            #""" Draw a yellow ellipse around the tracked region """
            #cv.EllipseBox(cv_image, track_box, cv.CV_RGB(255,255,0), 3, cv.CV_AA, 0 )
            
            self.ROI = RegionOfInterest()
            self.ROI.x_offset = int(min(self.image_size[0], max(0, camshift_box[0][0] - camshift_box[1][0] / 2)))
            self.ROI.y_offset = int(min(self.image_size[1], max(0, camshift_box[0][1] - camshift_box[1][1] / 2)))
            self.ROI.width = int(camshift_box[1][0])
            self.ROI.height = int(camshift_box[1][1])
                    
            self.pubROI.publish(self.ROI)
        
        """ Process any keyboard commands """
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'b':
                self.backproject_mode = not self.backproject_mode
                
        return camshift_box
        
        
#        if not self.backproject_mode:
#            return cv_image
#        else:
#            cv.CvtColor(self.backproject, self.backproject_rgb, cv.CV_GRAY2RGB)
#            return self.backproject_rgb
            
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
        
    def Ipl2NumPy(self, input):
        """Converts an OpenCV/IPL image to a numpy array.
    
        Supported input image formats are
           IPL_DEPTH_8U  x 1 channel
           IPL_DEPTH_8U  x 3 channels
           IPL_DEPTH_32F x 1 channel
           IPL_DEPTH_32F x 2 channels
           IPL_DEPTH_32S x 1 channel
           IPL_DEPTH_64F x 1 channel
           IPL_DEPTH_64F x 2 channels
        """
        
#        if not isinstance(input, cv.CvMat):
#            raise TypeError, 'must be called with a cv.CvMat!'
              
        # data type dictionary:
        # (channels, depth) : numpy dtype
        ipl2dtype = {
            (1, cv.IPL_DEPTH_8U)  : numpy.uint8,
            (3, cv.IPL_DEPTH_8U)  : numpy.uint8,
            (1, cv.IPL_DEPTH_32F) : numpy.float32,
            (2, cv.IPL_DEPTH_32F) : numpy.float32,
            (1, cv.IPL_DEPTH_32S) : numpy.int32,
            (1, cv.IPL_DEPTH_64F) : numpy.float64,
            (2, cv.IPL_DEPTH_64F) : numpy.float64
            }
        
        key = (input.nChannels, input.depth)
        if not ipl2dtype.has_key(key):
            raise ValueError, 'unknown or unsupported input mode'
        
        # Get the numpy array and reshape it correctly
        # ATTENTION: flipped dimensions width/height on 2007-11-15
        if input.nChannels == 1:
            array_1d = numpy.fromstring(input.imageData, dtype=ipl2dtype[key])
            return numpy.reshape(array_1d, (input.height, input.width))
        elif input.nChannels == 2:
            array_1d = numpy.fromstring(input.imageData, dtype=ipl2dtype[key])
            return numpy.reshape(array_1d, (input.height, input.width, 2))
        elif input.nChannels == 3:
            # Change the order of channels from BGR to RGB
            rgb = cv.cvCreateImage(cv.cvSize(input.width, input.height), input.depth, 3)
            cv.cvCvtColor(input, rgb, cv.CV_BGR2RGB)
            array_1d = numpy.fromstring(rgb.imageData, dtype=ipl2dtype[key])
            return numpy.reshape(array_1d, (input.height, input.width, 3))

        
    def NumPy2Ipl(self, input):
        """Converts a numpy array to the OpenCV/IPL CvMat data format.
    
        Supported input array layouts:
           2 dimensions of numpy.uint8
           3 dimensions of numpy.uint8
           2 dimensions of numpy.float32
           2 dimensions of numpy.float64
        """
        
        if not isinstance(input, numpy.ndarray):
            raise TypeError, 'Must be called with numpy.ndarray!'
    
        # Check the number of dimensions of the input array
        ndim = input.ndim
        if not ndim in (2, 3):
            raise ValueError, 'Only 2D-arrays and 3D-arrays are supported!'
        
        # Get the number of channels
        if ndim == 2:
            channels = 1
        else:
            channels = input.shape[2]
        
        # Get the image depth
        if input.dtype == numpy.uint8:
            depth = cv.IPL_DEPTH_8U
        elif input.dtype == numpy.float32:
            depth = cv.IPL_DEPTH_32F
        elif input.dtype == numpy.float64:
            depth = cv.IPL_DEPTH_64F
        
        # supported modes list: [(channels, dtype), ...]
        modes_list = [(1, numpy.uint8), (3, numpy.uint8), (1, numpy.float32), (1, numpy.float64)]
        
        # Check if the input array layout is supported
        if not (channels, input.dtype) in modes_list:
            raise ValueError, 'Unknown or unsupported input mode'
        
        result = cv.cvCreateImage(
            cv.cvSize(input.shape[1], input.shape[0]),  # size
            depth,  # depth
            channels  # channels
            )
        
        # set imageData
        result.imageData = input.tostring()
        
        return result

        
def main(args):
    # Display a help message if appropriate.
    help_message =  ""
          
    print help_message
    
    # Fire up the CamShift node.
    CS = CamShiftNode("camshift_filter")

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down vision node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)