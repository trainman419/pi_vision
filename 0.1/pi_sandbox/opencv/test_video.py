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

cv.NamedWindow("OpenCV Input", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("OpenCV Output", cv.CV_WINDOW_AUTOSIZE)


#img = cv.LoadImage("/home/patrick/Downloads/pi_robot_urdf.png")
#img_resized = cv.CreateImage((640, 480), 8, 3)
#cv.Resize(img, img_resized)
#img_out = doPyrDown(img_resized)
#img_out = doCanny(img_out, 70.0, 140.0, 3

video = cv.CaptureFromCAM(3)


while True:
    cv.GrabFrame(video)
    frame = cv.RetrieveFrame(video)
    cv.ShowImage("OpenCV Input", frame)
    frame = doPyrDown(frame)
    frame = doCanny(frame, 70.0, 140.0, 3)
    cv.ShowImage("OpenCV Output", frame)
    cv.WaitKey(10)
    
#cv.ShowImage("OpenCV Input", img_resized)
#cv.ShowImage("OpenCV Output", img_out)

#cv.MoveWindow("OpenCV", 400, 200)

cv.WaitKey()