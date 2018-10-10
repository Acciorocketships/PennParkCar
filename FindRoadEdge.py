from imgstream import Stream
import cv2
import numpy as np

class RoadFinder:

	def __init__(self):
		pass

	def filterColor(self,img):
		frontpoint = (0.5, 0.1) # fractions of total resolution in (x,y)
		avgsize = 5 # number of pixels to average
		ranges = (255, 20, 255) # threshold +/- this amount for hue and saturation
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





if __name__ == '__main__':
	roadfinder = RoadFinder()
	stream = Stream(mode='img',src='Files/Pictures')
	for img in stream:
		mask = roadfinder.filterColor(img)
		maskedimg = Stream.mask(mask,img,alpha=0.3)
		Stream.show(maskedimg,name="img",shape=(720,1280),pause=True)