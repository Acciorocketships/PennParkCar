from threading import Thread
from Filter import *
from ComputerVision import *
from ImgStream import *
from Map import *
from Planning import *
from math import *
from time import time, sleep
import numpy as np
from Message import Message


class MainLoop:

	def __init__(self):
		self.map = Map()
		self.message = Message()
		self.roadfinder = RoadFinder()
		self.planner = Planner()
		self.imgstream = Stream(mode='img',src='Files/Pictures')
		self.threads = {}
		# Variables
		self.inputs = {'psiIMUdot': 0, 'posGPS': np.array([0,0])}
		self.localvars = {'psiCAM': 0, 'psiIMU': 0, 'yposCAM': 0, 'vel': 0,
						  'psi': 0, 'pos': np.array([0,0]), 'side': None}
		self.globalvars = {'posGPS': np.array([0,0]), 'psiGPS': 0, 
						   'psiGPSconfidence': 0, 'posGPSconfidence': 0,
						   'psiRoad': 0, 'progress': 0, 'progressFrac': 0,
						   'intersection': False, 'nextRoad': np.array([0,0]),
						   'road', (None,None), 'roadStart': np.array([0,0]), 'roadEnd': np.array([0,0])}
		# Outputs
		self.psid = 0
		self.veld = 0
		# Options
		self.showimg = False
		self.print = False
		self.manual = False


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
					self.localvars['yposCAM'] = self.planner.desiredOffset - dist
					self.localvars['side'] = 'right'
				else:
					self.localvars['yposCAM'] = dist - self.planner.desiredOffset
					self.localvars['side'] = 'right'
				self.localvars['psiCAM'] = psi
			else:
				self.localvars['yposCAM'] = 0
				self.localvars['psiCAM'] = 0
				self.localvars['side'] = None


	def filter(self):
		lasttime = time()
		psiIMU = Integrator()
		psi = Filter(wc=1)
		xpos = Filter(wc=1)
		ypos = Filter(wc=1)

		while True:
			dt = time() - lasttime
			lasttime = time()
			# Psi Integrator
			psiIMU.update(self.inputs['psiIMUdot'],dt=dt)
			self.localvars['psiIMU'] = psiIMU.val
			# Psi
			psi.lowmeas = self.localvars['psiCAM']
			psi.highmeas = self.localvars['psiIMU']
			psi.update(dt=dt)
			self.localvars['psi'] = psi.val
			# Y Pos (From Road Edge)
			ypos.lowmeas = self.localvars['yposCAM']
			ypos.highmeas = self.localvars['vel'] * dt * sin(self.localvars['psi'])
			ypos.update(dt=dt)
			# X Pos (From Road Start)
			xpos.lowmeas = np.linalg.norm(self.globalvars['posGPS'] - self.globalvars['roadStart'])
			xpos.highmeas = self.localvars['vel'] * dt * cos(self.localvars['psi'])
			xpos.update(dt=dt)
			# Pos
			self.localvars['pos'] = np.array([xpos.val, ypos.val])


	def gps(self):
		looptime = 0.2
		gpsEMAlong = np.array([0,0])
		gpsEMAshort = np.array([0,0])
		longEMAalpha = 0.05 * looptime
		shortEMAalpha = 0.5 * looptime

		while True:
			mapData = self.map.predictPos(self.map.deg2meters(self.inputs['posGPS']))
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
			self.globalvars['progressFrac'] = mapData["Fraction Along Road"]
			self.globalvars['progress'] = mapData["Distance Along Road"]
			self.globalvars['intersection'] = ((self.globalvars['progressFrac'] * roadLength < self.planner.intersectionRadius) or
			 								  ((1-self.globalvars['progressFrac']) * roadLength < self.planner.intersectionRadius))



	def control(self):
		while True:
			if not self.manual:
				self.psid = self.planner.psid(self.localvars,self.globalvars)
				self.veld = self.map.speedLimit(self.globalvars['Road'][0],self.globalvars['Road'][1])


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










def str_dict(dictionary,spaces=0):
    string = ""
    if type(dictionary) == dict:
        for (key,value) in dictionary.items():
            if type(value) != dict and type(value) != list:
                string += " " * spaces*4 + " " + str(key) + " : " + str(value) + "\n"
            else:
                string += " " * spaces*4 + " " + str(key) + " : \n"
                string += str_dict(value,spaces+1)
    elif type(dictionary) == list:
        for item in dictionary:
            string += str_dict(item,spaces+1)
    else:
        string += " " * spaces*4 + " " + str(dictionary) + "\n"
    return string

