#!/usr/bin/env python

import roslib; roslib.load_manifest('pi_robot')
import rospy
import cv
import time
from sensor_msgs.msg import RegionOfInterest, CameraInfo

def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)

class CamShiftROS():
    def __init__(self):
        rospy.init_node("camshift")
        
        self.capture = cv.CaptureFromCAM(3)
        self.image_width = int(cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_WIDTH))
        self.image_height = int(cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_HEIGHT))
        cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS, 30)
        self.fps = int(cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS))

        
        rospy.loginfo(str(self.image_width) + " x " + str(self.image_height) + " at " + str(self.fps) + " fps")

        cv.NamedWindow( "CamShiftRGB", 1 )
        cv.NamedWindow( "Histogram", 1 )
        cv.MoveWindow( "Histogram", 700, 10)
        cv.SetMouseCallback( "CamShiftRGB", self.on_mouse)

        self.drag_start = None      # Set to (x,y) when mouse starts dragtime
        self.track_window = None    # Set to rect when the mouse drag finishes
        
        self.rate = int(rospy.get_param("~rate", 20))
        r = rospy.Rate(self.rate)

        self.ROI = rospy.Publisher("ROI", RegionOfInterest)
        self.CameraInfo = rospy.Publisher("CameraInfo", CameraInfo)

        print( "Keys:\n"
            "    ESC - quit the program\n"
            "    b - switch to/from backprojection view\n"
            "To initialize tracking, drag across the object with the mouse\n" )
        
        hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )
        backproject_mode = False
        
        while not rospy.is_shutdown():
            frame = cv.QueryFrame( self.capture )
            image_size = cv.GetSize(frame)
            image_width = image_size[0]
            image_height = image_size[1]
            
            camera_info = CameraInfo()
            camera_info.width = image_width
            camera_info.height = image_height
            self.CameraInfo.publish(camera_info)      

            # Convert to HSV and keep the hue
            hsv = cv.CreateImage(image_size, 8, 3)
            cv.CvtColor(frame, hsv, cv.CV_BGR2HSV)
            self.hue = cv.CreateImage(image_size, 8, 1)
            cv.Split(hsv, self.hue, None, None, None)

            # Compute back projection
            backproject = cv.CreateImage(image_size, 8, 1)

            # Run the cam-shift
            cv.CalcArrBackProject( [self.hue], backproject, hist )
            if self.track_window and is_rect_nonzero(self.track_window):
                crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
                (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.track_window, crit)
                self.track_window = rect

            # If mouse is pressed, highlight the current selected rectangle
            # and recompute the histogram

            if self.drag_start and is_rect_nonzero(self.selection):
                sub = cv.GetSubRect(frame, self.selection)
                save = cv.CloneMat(sub)
                cv.ConvertScale(frame, frame, 0.5)
                cv.Copy(save, sub)
                x,y,w,h = self.selection
                cv.Rectangle(frame, (x,y), (x+w,y+h), (255,255,255))

                sel = cv.GetSubRect(self.hue, self.selection )
                cv.CalcArrHist( [sel], hist, 0)
                (_, max_val, _, _) = cv.GetMinMaxHistValue( hist)
                if max_val != 0:
                    cv.ConvertScale(hist.bins, hist.bins, 255. / max_val)
            elif self.track_window and is_rect_nonzero(self.track_window):
                cv.EllipseBox( frame, track_box, cv.CV_RGB(255,0,0), 3, cv.CV_AA, 0 )
                msg = RegionOfInterest()
                msg.x_offset = int(min(image_width, max(0, track_box[0][0] - track_box[1][0] / 2)))
                msg.y_offset = int(min(image_height, max(0, track_box[0][1] - track_box[1][1] / 2)))
                msg.width = int(track_box[1][0])
                msg.height = int(track_box[1][1])
                self.ROI.publish(msg)

            if not backproject_mode:
                cv.ShowImage( "CamShiftRGB", frame )
            else:
                cv.ShowImage( "CamShiftRGB", backproject)
            cv.ShowImage( "Histogram", self.hue_histogram_as_image(hist))

            c = cv.WaitKey(7) % 0x100
            if c == 27:
                break
            elif c == ord("b"):
                backproject_mode = not backproject_mode
            
            r.sleep()

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

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.track_window = self.selection
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)


if __name__ == '__main__':
    try:
        demo = CamShiftROS()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down camshift node...")

