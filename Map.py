from queue import PriorityQueue
import math
import numpy as np

class Map:


	def __init__(self):

		# Read in Edges
		edgeFile = open("Files/Map/mapPennParkEdges.txt","r")
		self.edges = {}
		self.maxspeed = 0
		for line in edgeFile:
			data = line.split()
			if data[0] not in self.edges:
				self.edges[data[0]] = {}
			self.edges[data[0]][data[1]] = np.array([float(data[2]),float(data[3])])
			self.maxspeed = max(self.maxspeed,float(data[2]),float(data[3]))
		edgeFile.close()

		# Read in Nodes
		nodeFile = open("Files/Map/mapPennParkNodes.txt","r")
		self.nodes = {}
		for line in nodeFile:
			data = line.split()
			self.nodes[data[0]] = np.array([float(data[1]),float(data[2])])
		nodeFile.close()

		firstnode = self.nodes["A"]
		self.basepos = np.array([firstnode[0],firstnode[1]])
		self.fy = 111133 - 560*math.cos(2*firstnode[0] * 3.1416/180)
		self.fx = 111413*math.cos(firstnode[0] * 3.1416/180) - 93.5*math.cos(3*firstnode[0] * 3.1416/180)


	def speedLimit(self,start,end):
		if start is None or end is None:
			return 0
		direction = (start < end)
		if direction:
			if (start in self.edges) and (end in self.edges[start]):
				return self.edges[start][end][0]
		else:
			if (end in self.edges) and (start in self.edges[end]):
				return self.edges[end][start][1]
		return 0


	def deg2meters(self,pos,relative=True):

		if relative:
			pos = pos - self.basepos
		pos = np.array([pos[1]*self.fx, pos[0]*self.fy]) # (x, y)
		return pos


	def meters2deg(self,pos,absolute=True):

		pos = np.array([pos[1]/self.fy, pos[0]/self.fx]) # (lat, lng)
		if absolute:
			pos = pos + self.basepos
		return pos


	def roads(self):

		for start, edge in self.edges.items():
			for end, speeds in edge.items():
				yield (start, end)


	def predictPos(self,pos,degrees=False):

		if degrees:
			pos = self.deg2meters(pos)

		minerr = float("inf")
		bestpred = None
		bestroad = None
		roadstart = None
		roadend = None
		bestroaddist = None
		bestroadfrac = None
		for road in self.roads():
			start = self.deg2meters(self.nodes[road[0]])
			end = self.deg2meters(self.nodes[road[1]])
			direction = start - end
			direction = direction / np.linalg.norm(direction)
			predicted = direction * np.dot(direction, (pos - start)) + start
			error = np.linalg.norm(predicted - pos)
			roaddist0 = np.linalg.norm(predicted - start)
			roadfrac0 = roaddist0 / np.linalg.norm(end - start)
			roaddist1 = np.linalg.norm(predicted - end)
			roadfrac1 = roaddist1 / np.linalg.norm(end - start)
			if error < minerr and roadfrac0 <= 1 and roadfrac1 <= 1:
				minerr = error
				bestpred = predicted
				bestroad = road
				roadstart = start
				roadend = end
				bestroaddist = roaddist0
				bestroadfrac = roadfrac0

		if degrees:
			bestpred = self.meters2deg(bestpred)
			roadstart = self.meters2deg(roadstart)
			roadend = self.meters2deg(roadend)

		return {"Prediction": bestpred, 
				"Road": bestroad,
				"Road Start": roadstart,
				"Road End": roadend,
				"Error": minerr, 
				"Distance Along Road": bestroaddist,
				"Fraction Along Road": bestroadfrac}


	def time(self,node1,node2):

		node1pos = self.deg2meters(node1)
		node2pos = self.deg2meters(node2)
		dist = (node1pos[0]-node2pos[0])**2 + (node1pos[1]-node2pos[1])**2
		speed = self.speedLimit(node1,node2)
		speed = speed if (speed != 0) else self.maxspeed
		time = dist / speed
		return time


	def next(self,node):

		nextnodes = []
		if node in self.edges:
			for nextnode, speed in self.edges[node].items():
				if speed != 0:
					nextnodes.append(nextnode)
		return nextnodes


	def pathplan(self,start,end):

		q = PriorityQueue()
		prev = {}
		for node in self.next(start):
			time = self.time(start,node)
			heuristic = self.time(node,end)
			q.put((time,node))
			prev[node] = (start,time)

		while not q.empty():
			pathtime, curr = q.get()
			if curr == end:
				break
			for node in self.next(curr):
				time = self.time(curr,node) + pathtime
				heuristic = self.time(node,end)
				q.put((time+heuristic,node))
				if node not in prev or prev[node][1] > time:
					prev[node] = (curr,time)

		node = end
		path = [end]
		while node != start:
			node = prev[node]
			path.append(node)
		path = path.reverse()
		return path




def showpts(m):

	from matplotlib import pyplot as plt
	plt.figure(0)
	xm = []
	ym = []
	for node, pos in m.nodes.items():
		posm = m.deg2meters(pos)
		xm.append(posm[0])
		ym.append(posm[1])
	plt.scatter(np.array(xm),np.array(ym))
	plt.show()


if __name__ == '__main__':
	m = Map()
	a = m.predictPos(m.meters2deg(np.array([-30,-50])), degrees=True)
	print(a)
	# import code
	# code.interact(local=locals())