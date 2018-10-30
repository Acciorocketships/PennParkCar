from threading import Thread
from Filter import *
from FindRoadEdge import *
from ImgStream import *
from NearestRoad import *
from math import *
from time import time, sleep

class MainLoop:

	def __init__(self):
		self.map = Map()
		self.roadfinder = RoadFinder()
		self.showimg = False
		self.imgstream = Stream(mode='img',src='Files/Pictures')
		self.threads = {}
		# Variables
		self.vars = {'psiCAM': 0, 'psiIMU': 0, 'psiRoad': 0, 'posCAM': 0, 'posGPS': 0, 'vel': 0}
		self.localpos = 0
		self.localpsi = 0
		# Options
		self.desiredEdgeDist = 1


	def run(self):
		visionThread = Thread(target=self.findroads)
		self.threads['vision'] = visionThread
		visionThread.start()


	def vision(self):
		for img in imgstream:
			# Process Image
			edge = self.roadfinder.find(img,show=self.showimg) # edge = ((x1,y1),(x2,y2))
			# Get Distance from edge and Psi relative to edge
			dist = (edge[1][0]*edge[0][1] + edge[1][1]*edge[0][0]) / ( (edge[1][1]-edge[0][1])**2 + (edge[1][0]-edge[0][0])**2 )
			psi = -atan2(edge[1][0]-edge[0][0], edge[1][1]-edge[0][1])
			psi = psi + pi if (psi < -pi/2) else psi - pi if (psi > pi/2) else psi # normalize
			line = lambda point: tan(psi)*(point[1]-edge[0][1]) + edge[0][0] # x = 1/m * (y-y1) + x1
			if x > line((0,0)):
				self.vars['posCAM'] = self.desiredEdgeDist - dist
			else:
				self.vars['posCAM'] = dist - self.desiredEdgeDist
			self.vars['psiCAM'] = psi


	def filter(self):
		lasttime = time()
		psi = Filter(wc=1)
		pos = Filter(wc=1)
		while True:
			dt = time() - lasttime
			lasttime = time()
			# Psi
			psi.lowmeas = self.vars['psiCAM']
			psi.highmeas = self.vars['psiIMU']
			psi.update(dt=dt)
			self.localpsi = psi.val
			pos.lowmeas = self.vars['posCAM']
			pos.highmeas = (self.vars['vel'] * dt) * np.array([cos(self.localpsi), sin(self.localpsi)])

