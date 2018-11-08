from threading import Thread
from Filter import *
from FindRoadEdge import *
from ImgStream import *
from Map import *
from math import *
from time import time, sleep
import numpy as np
from Message import Message


class MainLoop:

	def __init__(self):
		self.map = Map()
		self.message = Message()
		self.roadfinder = RoadFinder()
		self.imgstream = Stream(mode='img',src='Files/Pictures')
		self.threads = {}
		# Variables
		self.inputs = {'psiIMUdot': 0, 'posGPS': (0,0)}
		self.localvars = {'psiCAM': 0, 'psiIMU': 0, 'posCAM': 0, 'vel': 0,
						  'psi': 0, 'pos': 0, 'side': None}
		self.globalvars = {'posGPS': 0, 'psiGPS': 0, 
						   'psiGPSconfidence': 0, 'posGPSconfidence': 0,
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
		# if high confidence and going in the opposite direction, then turn around
		looptime = 0.2
		roadMap = Map()
		gpsEMAlong = np.array([0,0])
		gpsEMAshort = np.array([0,0])
		longEMAalpha = 0.05 * looptime
		shortEMAalpha = 0.5 * looptime

		while True:
			mapData = roadMap.predictPos(roadMap.deg2meters(self.inputs['posGPS']))
			self.globalvars['roadStart'] = mapData['Road Start']
			self.globalvars['roadEnd'] = mapData['Road End']
			self.globalvars['posGPS'] = mapData['Prediction']
			gpsEMAlong = longEMAalpha * self.globalvars['posGPS'] + (1-longEMAalpha) * gpsEMAlong
			gpsEMAshort = shortEMAalpha * self.globalvars['posGPS'] + (1-shortEMAalpha) * gpsEMAshort
			vecGPS = gpsEMAshort - gpsEMAlong
			self.globalvars['psiGPS'] = atan2(vecGPS[1],vecGPS[0])
			vecRoad = mapData['Road Start'] - mapData['Road End']
			roadLength = np.linalg.norm(vecRoad)
			vecRoad = vecroad / roadLength
			self.globalvars['psiGPSconfidence'] = tanh(0.1*np.dot(vecGPS,vecRoad))
			self.globalvars['posGPSconfidence'] = exp(-0.1*mapData['Error'])
			self.globalvars['progress'] = mapData["Fraction Along Road"]
			self.globalvars['intersection'] = ((self.globalvars['progress'] * roadLength < intersectionRadius) or
			 								  ((1-self.globalvars['progress']) * roadLength < intersectionRadius))



	def receive(self):
		self.message.recieve()
		self.inputs['psiIMUdot'] = self.message.gyroz
		self.inputs['posGPS'] = (self.message.gpsLat, self.message.gpsLon)


	def send(self):
                self.message.manual = False
		self.message.desHeading = self.psid
		self.message.desSpeed = self.veld
		self.message.estHeading = self.localvars['psi']
		self.message.estSpeed = self.localvars['vel']
                self.message.send()

