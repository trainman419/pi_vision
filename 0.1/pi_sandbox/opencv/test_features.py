import cv

#cv.NamedWindow("OpenCV Input", cv.CV_WINDOW_AUTOSIZE)

img = cv.LoadImageM("/home/patrick/Downloads/pi_robot_urdf.png", cv.CV_LOAD_IMAGE_GRAYSCALE)

eig_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)
temp_image = cv.CreateMat(img.rows, img.cols, cv.CV_32FC1)

for (x,y) in cv.GoodFeaturesToTrack(img, eig_image, temp_image, 30, 0.04, 50, useHarris = True):
    cv.Circle(img, (int(x), int(y)), 10, cv.RGB(0, 255, 255), thickness=10)
    print "good feature at", x,y
    
cv.ShowImage("OpenCV Output", img)
    
cv.WaitKey()

  