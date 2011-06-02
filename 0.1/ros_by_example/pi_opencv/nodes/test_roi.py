#!/usr/bin/python

import cv

# Load image
image = cv.LoadImageM("lena.jpg", 1)

#image = cv.CreateImage(cv.GetSize(image), 8, 3)

roi = cv.CreateImage(cv.GetSize(image), 8, 1)
result_image = cv.CreateImage(cv.GetSize(image), 8, 3)
grey_image = cv.CreateImage(cv.GetSize(image), 8, 1)
sub_image = cv.CreateImage(cv.GetSize(image), 8, 3)

cv.Zero(roi)

cv.Circle(roi, (200,200), 50, cv.CV_RGB(255,255, 255), -1, 8, 0)

#cv.And(image, image, result_image, roi)

cv.CvtColor(image, grey_image, cv.CV_RGB2GRAY) 
eig_image = cv.CreateImage(cv.GetSize(grey_image), cv.IPL_DEPTH_32F, 1)
temp_image = cv.CreateImage(cv.GetSize(grey_image), cv.IPL_DEPTH_32F, 1)

pts = cv.GoodFeaturesToTrack(grey_image, eig_image, temp_image, 100, 0.09, 2, roi, useHarris=1)
for point in pts:
    cv.Line(image, point, point, cv.RGB(0,255,255), 3)


#sub = cv.GetSubRect(result, (200, 200, 100, 100))


#cv.CvtColor(img, grey, cv.CV_RGB2GRAY)
#corners = cv.CloneImage(img)
#cv.CvtColor(corners, corners, cv.CV_32FC1)
#edges = cv.CloneImage(img)

#edges = cv.CreateImage(cv.GetSize(img), 8, 1)


# Set image ROI
#cv.SetImageROI(grey, (200, 200, 150, 250))
#cv.SetImageROI(edges, (200, 200, 150, 250))

# Do some processing here
# In this case, simply invert the subimage
#cv.Canny(img, edges, 0.1, 5.0, aperture_size=5)
#cv.CornerHarris(img, corners, 12)

# Clear ROI
#cv.ResetImageROI(grey)
#cv.ResetImageROI(edges)


# Display image
cv.NamedWindow("image", 1)
cv.ShowImage("image", image)
cv.WaitKey(0)

def get_sample(self, filename, iscolor = cv.CV_LOAD_IMAGE_COLOR):
    filedata = urllib.urlopen("file://" + filename).read()
    imagefiledata = cv.CreateMatHeader(1, len(filedata), cv.CV_8UC1)
    cv.SetData(imagefiledata, filedata, len(filedata))
    return cv.DecodeImageM(imagefiledata, iscolor)

