from imgstream import Stream
import cv2
import numpy as np
from math import *

class RoadFinder:

	def __init__(self):
		pass

	def filterColor(self, img, ranges=(255,25,255)):
		frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
		avgsize = 5 # number of pixels to average
		blursize = 25
		# Calculations
		imghsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		imghsv = cv2.GaussianBlur(imghsv,(blursize,blursize),0)
		frontpoint = (int(imghsv.shape[0]*(1-frontpoint[1])), int(imghsv.shape[1]*frontpoint[0]))
		frontregion = imghsv[frontpoint[0]-avgsize//2:frontpoint[0]+avgsize//2, frontpoint[1]-avgsize//2:frontpoint[1]+avgsize//2, 0:3]
		avgcolor = np.average(np.average(frontregion,axis=1),axis=0).flatten()
		lowerthres = np.array([max(avgcolor[0]-ranges[0], 0), max(avgcolor[1]-ranges[1], 0), max(avgcolor[2]-ranges[2], 0)], dtype=np.uint8)
		upperthres = np.array([min(avgcolor[1]+ranges[0], 255), min(avgcolor[1]+ranges[1], 255), min(avgcolor[2]+ranges[2], 255)], dtype=np.uint8)
		mask = cv2.inRange(imghsv, lowerthres, upperthres)
		# mask = cv2.GaussianBlur(mask,(blursize,blursize),0)
		return mask

	# def filterBottom(self,img):
	# 	bottomfrac = 0.7
	# 	mask = np.zeros((img.shape[0],img.shape[1]))
	# 	mask[int((1-bottomfrac)*img.shape[0]):img.shape[0],:] = 1
	# 	return mask


	def filter(self, img):
		mask = self.filterColor(img)
		return mask


	def hull(self,mask,draw=None):
		# Get contours
		mask.dtype = np.uint8
		im2, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		# Get largest contour
		maxarea = 0
		maxcontour = contours[0]
		for i, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if area > maxarea:
				maxarea = area
				maxcontour = contour

		# Simplify contour
		epsilon = 150
		simplecontour = cv2.approxPolyDP(maxcontour,epsilon,True)

		# Visualization
		if draw is not None:
			cv2.drawContours(draw, [maxcontour], 0, (255,0,0), 8)
			cv2.drawContours(draw, [simplecontour], 0, (0,0,255), 8)

		return simplecontour


	def edge(self,hull):

		# Find best segment in the hull
		minlength = 1200
		best = 0
		segment = None
		segangle = 0
		for i in range(1,hull.shape[0]):
			# Check that line is not horizontal or vertical
			if (hull[i,0,0] != hull[i-1,0,0]) and (hull[i,0,1] != hull[i-1,0,1]):
				# Check that the line is above a threshold length
				length = np.linalg.norm(hull[i,0,:]-hull[i-1,0,:])
				if length > minlength:
					# Get angle
					angle = -atan2(hull[i,0,0] - hull[i-1,0,0], hull[i,0,1] - hull[i-1,0,1])
					if angle > pi/2:
						angle -= pi
					elif angle < -pi/2:
						angle += pi
					# Choose best segment according to quality function
					quality = cos(angle) * length
					if quality > best:
						best = quality
						segment = (hull[i-1,0,:], hull[i,0,:])
						segangle = angle
		return segment


		

if __name__ == '__main__':
	roadfinder = RoadFinder()
	stream = Stream(mode='img',src='Files/Pictures')
	for img in stream:
		mask = roadfinder.filter(img)
		hull = roadfinder.hull(mask, draw=img)
		edge = roadfinder.edge(hull)
		maskedimg = Stream.mask(mask,img,alpha=0.3)
		maskedimg = Stream.mark(maskedimg, (tuple(edge[0]),tuple(edge[1])), color=(0,255,0))
		Stream.show(maskedimg,name="img",shape=(720,1280),pause=True)


