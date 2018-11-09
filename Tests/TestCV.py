import picamera
import cv2
import numpy as np
#
# set up the camera
# default photo seems to be 1920 x 1080
# half of that keeps things more manageable for on-screen debugging
#
camera = picamera.PiCamera()
photoHeight = 540
camera.resolution = (16*photoHeight/9, photoHeight)
#
# captue an image and read it back in
# (Do this because picamera does not play nice with openCV?)
#
camera.capture('blackRoad.jpg')
imgColor = cv2.imread('blackRoad.jpg') 
#
# convert to grayscale -- this seems to be standard for edge detection
# but we may ultimately want to do something more fancy to help us find
# the border between road and grass
#
img = cv2.cvtColor(imgColor, cv2.COLOR_BGR2GRAY)
#
# blur is standard to ensure that the edge goes along the entire
# length of the edge--noisy pixels on the edge can be excluded otherwise
# we may also want this to eliminate the texture of the road
#
img = cv2.GaussianBlur(img,(3,3),0)
#
# "Canny" is the guy who invented the edge detection algorithm
# that is very widely used.
# the two arguments are the thresholds used in the algorithm
#
edges = cv2.Canny(img,20,80)
#
# show the edges and save the edges image
# note that edges is effectively black-and-white, with white only
# along the edges
#
cv2.imshow('Edges',edges)
cv2.imwrite('edges.jpg',edges)
#
# now look for lines with the Hough algorithm
# theta = resolution for slopes of lines (increments)
# threshold = how many points share the same line?
#
minLineLength = 50
maxLineGap = 10
lines = cv2.HoughLinesP(edges,rho=1,theta=np.pi/180,threshold=50,minLineLength=minLineLength,maxLineGap=maxLineGap)
#
# now parse through the lines that were found
# WE NEED TO IGNORE MOST OF THESE LINES
# AND ADD CODE TO EXTRACT RELATIVE HEADING AND OFFSET
#
for x in range(0, len(lines)):
    for x1,y1,x2,y2 in lines[x]:
        cv2.line(imgColor,(x1,y1),(x2,y2),(0,0,255),2) # draw the line on the original image
#
# show the original image with the lines
#
cv2.imshow('hough',imgColor)
cv2.imwrite('hough.jpg',imgColor)
#
# the graphics windows opened by CV2 seem to freak out
# if you don't have this command at the end
#
cv2.waitKey(0)
