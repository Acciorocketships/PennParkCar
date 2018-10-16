from imgstream import Stream
import cv2
import numpy as np
from math import *
import time

class RoadFinder:

	def __init__(self):
		self.setRT()
		self.setCam()


	def setCam(self,focal=3.5E-3,fov=65,res=(720,1280)):
		self.camFocal = focal
		self.res = res
		self.pix2dist = (focal * tan(pi/180*fov/2)) / (res[1] / 2)


	def setRT(self,heightoffset=0.1,sideoffset=0,forwardoffset=0,yawoffset=0,pitchoffset=0,rolloffset=0):
		self.camT = np.array([forwardoffset, heightoffset, sideoffset]) # x (forward), y (up), z (side)
		invyaw = np.array([[cos(-yawoffset), -sin(-yawoffset), 0],[sin(-yawoffset), cos(-yawoffset), 0],[0, 0, 1]])
		invpitch = np.array([[cos(-pitchoffset), 0, sin(-pitchoffset)],[0, 1, 0],[-sin(-pitchoffset), 0, cos(-pitchoffset)]])
		invroll = np.array([[1, 0, 0],[0, cos(-rolloffset), -sin(-rolloffset)],[0, sin(-rolloffset), cos(-rolloffset)]])
		self.camR = invroll * invpitch * invyaw


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


	def filterBottom(self,img):
		bottomfrac = 0.7
		mask = np.zeros((img.shape[0],img.shape[1]))
		mask[int((1-bottomfrac)*img.shape[0]):img.shape[0],:] = 1
		return mask


	def filter(self, img):
		mask1 = self.filterBottom(img)
		mask2 = self.filterColor(img)
		# TODO: dynamics filtering, decreases range when the top of the img is active,
		#       increases range when the bottom is active
		mask = np.logical_and(mask1,mask2)
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
		epsilon = 0.06*self.res[0]
		simplecontour = cv2.approxPolyDP(maxcontour,epsilon,True)

		# Visualization
		if draw is not None:
			cv2.drawContours(draw, [maxcontour], 0, (255,0,0), 8)
			cv2.drawContours(draw, [simplecontour], 0, (0,0,255), 8)

		return simplecontour


	def segment(self,hull):
		# Find best segment in the hull
		minlength = self.res[0]/2.5
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
					quality = cos(angle/2) * length
					if quality > best:
						best = quality
						segment = (hull[i-1,0,:], hull[i,0,:])
						segangle = angle
		# returns tuple of 2 element np arrays in img coordinates. transform to xy with self.img2xy(points)
		return segment


	def img2xy(self,points):
		# input point has origin in the top left, (x,y) order, increasing down and to the right
		# output point has origin in the center, (x,y) order, increasing up and to the right
		if points is None:
			return None
		output = []
		if type(points) != list and type(points) != tuple:
			points = [points]
		for point in points:
			point = np.array([point[0] - self.res[1]/2, self.res[0]*3/2 - point[1]])
			output.append(point)
		if type(points) == tuple:
			output = tuple(output)
		return output


	def transform(self,points,planeN=[0,1,0],planeT=[0,0,0]):
		if points is None:
			return None
		planeN = np.reshape(planeN,(3,))
		planeT = np.reshape(planeT,(3,))
		output = []
		# import pdb; pdb.set_trace()
		if type(points) != list and type(points) != tuple:
			points = [points]
		for point in points:
			point = np.reshape(point,(2,))
			point = np.concatenate((point * self.pix2dist, [self.camFocal]), axis=0)
			point = self.camR @ point # rotate to the world frame
			c = (-np.dot(self.camT,planeN) + np.dot(planeT,planeN)) / np.dot(point,planeN)
			transformed = c * point + self.camT
			output.append(transformed)
		return output


		

if __name__ == '__main__':
	roadfinder = RoadFinder()
	stream = Stream(mode='img',src='Files/Pictures')
	resolution = (720,1280)
	roadfinder.setCam(res=resolution)
	for img in stream:
		img = Stream.resize(img,resolution)
		imgshow = np.copy(img)
		starttime = time.time()
		mask = roadfinder.filter(img)
		hull = roadfinder.hull(mask, draw=imgshow)
		edge = roadfinder.segment(hull)
		worldedge = roadfinder.transform(roadfinder.img2xy(edge))
		print(worldedge)
		dtime = time.time() - starttime
		# print(dtime)
		maskedimg = Stream.mask(mask,imgshow,alpha=0.3)
		if worldedge is not None:
			maskedimg = Stream.mark(maskedimg, (tuple(edge[0]),tuple(edge[1])), color=(0,255,0))
		Stream.show(maskedimg,name="img",shape=(720,1280),pause=True)


