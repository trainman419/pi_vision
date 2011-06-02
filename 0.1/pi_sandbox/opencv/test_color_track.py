import cv

def doPyrDown(img):
    if img.width % 2 != 0 or img.height % 2 != 0:
        print "PyrDown requires input width and height to be multiples of 2."
        return False
    out = cv.CreateImage((img.width / 2, img.height  / 2), img.depth, img.nChannels)
    cv.PyrDown(img, out)
    return out

def doCanny(img, low_thresh, high_thresh, aperature):
    gray = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)

    if gray.nChannels != 1:
        print "Canny filter requires a grayscale input image."
        return False

    out = cv.CreateImage(cv.GetSize(gray), cv.IPL_DEPTH_8U, 1)
    cv.Canny(gray, out, low_thresh, high_thresh, aperature)
    return out

def doSobel(img, xorder, yorder, apertureSize = 3):
#    gray = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
#    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
#
#    if gray.nChannels != 1:
#        print "Sobel filter requires a grayscale input image."
#        return False

    out = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_16S, img.nChannels)
    cv.Sobel(img, out, xorder, yorder, apertureSize = 3)
    return out

def doEqualizeHist(img):
    gray = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
    out = cv.CreateImage(cv.GetSize(gray), gray.depth, gray.nChannels)
    cv.EqualizeHist(gray, out)
    return out

def doGoodFeaturesToTrack(img):
    gray = cv.CreateImage(cv.GetSize(img), cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
    out = cv.CreateImage(cv.GetSize(gray), gray.depth, gray.nChannels)
    cv.EqualizeHist(gray, out)
    return out

def doRGBFilter(img, r, g, b):
    out = cv.CreateImage(cv.GetSize(gray), cv.IPL_DEPTH_8U, 1)
    cv.Canny(gray, out, low_thresh, high_thresh, aperature)
    return out

cv.NamedWindow("OpenCV Input", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("OpenCV Output", cv.CV_WINDOW_AUTOSIZE)

#img = cv.LoadImage("/home/patrick/Downloads/pi_robot_urdf.png")
#img_resized = cv.CreateImage((640, 480), 8, 3)
#cv.Resize(img, img_resized)
#img_out = doPyrDown(img_resized)
#img_out = doCanny(img_out, 70.0, 140.0, 3

video = cv.CaptureFromCAM(3)

min_color = (100, 100, 100)
max_color = (255, 255, 255)

while True:
    cv.GrabFrame(video)
    frame = cv.RetrieveFrame(video)
    sobel = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_16S, frame.nChannels)
    out = cv.CreateImage(cv.GetSize(frame), frame.depth, frame.nChannels)

    #frame = doPyrDown(frame)
    #color_mask = cv.CreateImage(cv.GetSize(frame), 8, 1)
    #cv.InRangeS(frame, cv.Scalar(*min_color), cv.Scalar(*max_color), color_mask)
    
    #storage = cv.CreateMemStorage(0)
    #contours = cv.FindContours(color_mask, storage, method=cv.CV_CHAIN_APPROX_SIMPLE)
    #contours.h_next
    #print "N Contours:", len(contours)
#    cv.DrawContours(frame, contours, cv.RGB(255, 255, 0), cv.RGB(0, 0, 255), 2)
#    for x, y in contours:
#        print x, y
#        # Get the size of the contour
#        #size = abs(cv.ContourArea(contour))
#
#        # Is it convex?
#        #is_convex = cv.CheckContourConvexity(contour)
#s
#        # Find the bounding-box of the contour
#        bbox = cv.BoundingRect(points, 0)
#
#        # Calculate the x and y coordinate of center
#        x, y = bbox.x + bbox.width * 0.5, bbox.y + bbox.height * 0.5
#        print x, y


    #cv.ShowImage("OpenCV Input", frame)
    #frame = doEqualizeHist(frame)
    
#    eig_image = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
#    temp_image = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
#    gray = cv.CreateImage(cv.GetSize(frame), cv.IPL_DEPTH_8U, 1)
#    cv.CvtColor(frame, gray, cv.CV_BGR2GRAY)
#
#    for (x,y) in cv.GoodFeaturesToTrack(gray, eig_image, temp_image, 30, 0.1, 50, useHarris = True):
#        cv.Circle(frame, (int(x), int(y)), 6, cv.RGB(0, 255, 255), thickness=3)
        #print "good feature at", x,y
    
    #frame = doCanny(frame, 70.0, 140.0, 3)
    sobel = doSobel(frame, 1, 1, 7)
    cv.ConvertScale(sobel, out)
    cv.ShowImage("OpenCV Input", frame)
    cv.ShowImage("OpenCV Output", out)
    cv.WaitKey(10)

#cv.ShowImage("OpenCV Input", img_resized)
#cv.ShowImage("OpenCV Output", img_out)

#cv.MoveWindow("OpenCV", 400, 200)
