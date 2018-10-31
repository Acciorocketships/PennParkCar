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
		self.imgstream = Stream(mode='img',src='Files/Pictures')
		self.threads = {}
		# Variables
		self.localvars = {'psiCAM': 0, 'psiIMU': 0, 'posCAM': 0, 'vel': 0,
						  'psi': 0, 'pos': 0, 'side': None}
		self.globalvars = {'posGPS': 0, 'psiGPS': 0, 'psiGPSconfidence': 0, 
						   'psiRoad': 0, 'progress': 0, 'intersection': False,
						   'roadStart': np.array([0,0]), 'roadEnd': np.array([0,0])}
		# Outputs
		self.psid = 0
		self.veld = 0
		# Options
		self.desiredEdgeDist = 1
		self.intersectionRadius = 5
		self.showimg = False
		self.print = False


	def run(self):
		visionThread = Thread(target=self.findroads)
		self.threads['vision'] = visionThread
		visionThread.start()


	def vision(self):
		for img in imgstream:
			# Process Image
			edge = self.roadfinder.find(img,show=self.showimg) # edge = ((x1,y1),(x2,y2))
			# Get Distance from edge and Psi relative to edge
			if edge is not None:
				dist = (edge[1][0]*edge[0][1] + edge[1][1]*edge[0][0]) / ( (edge[1][1]-edge[0][1])**2 + (edge[1][0]-edge[0][0])**2 )
				psi = -atan2(edge[1][1]-edge[0][1], edge[1][0]-edge[0][0])
				psi = psi + pi if (psi < -pi/2) else psi - pi if (psi > pi/2) else psi # normalize
				line = lambda point: tan(psi)*(point[1]-edge[0][1]) + edge[0][0] # x = 1/m * (y-y1) + x1
				if line((0,0)) > 0:
					self.localvars['posCAM'] = self.desiredEdgeDist - dist
					self.localvars['side'] = 'right'
				else:
					self.localvars['posCAM'] = dist - self.desiredEdgeDist
					self.localvars['side'] = 'right'
				self.localvars['psiCAM'] = psi
			else:
				self.localvars['posCAM'] = 0
				self.localvars['psiCAM'] = 0
				self.localvars['side'] = None


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
			self.localvars['psi'] = psi.val
			pos.lowmeas = self.vars['posCAM']
			pos.highmeas = self.vars['vel'] * dt * sin(self.localvars['psi'])
			pos.update(dt=dt)
			self.localvars['pos'] = pos.val


	def gps(self):
		# NearestRoad to rectify gps pos and calculate other globalvars
		# calculate long and short EMA of gps. subtract to get vector. direction is psi, and magnitude is confidence.
		# perhaps abs value of dot product of vector with road direction for confidence?
		pass


	def receive(self):
		pass


	def send(self):
		pass

