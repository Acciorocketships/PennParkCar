from threading import Thread
from Filter import *
from FindRoadEdge import *
from ImgStream import *
from NearestRoad import *

class MainLoop:

	def __init__(self):
		self.map = Map()
		self.roadfinder = RoadFinder()
		self.showimg = False
		self.imgstream = Stream(mode='img',src='Files/Pictures')
		self.threads = {}
		# Variables
		self.edge

	def run(self):
		visionThread = Thread(target=self.findroads)
		self.threads['vision'] = visionThread
		visionThread.start()

	def vision(self):
		for img in imgstream:
			edge = self.roadfinder.find(img,show=self.showimg)
	
