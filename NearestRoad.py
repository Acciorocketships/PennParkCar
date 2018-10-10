import math
import numpy as np

class Map:


	def __init__(self):

		# Read in Edges
		edgeFile = open("Files/Map/mapPennParkEdges.txt","r")
		self.edges = {}
		for line in edgeFile:
			data = line.split()
			if data[0] not in self.edges:
				self.edges[data[0]] = {}
			self.edges[data[0]][data[1]] = np.array([float(data[2]),float(data[3])])
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


def str_dict(dictionary,spaces=0):
    string = ""
    if type(dictionary) == dict:
        for (key,value) in dictionary.items():
            if key != 'photos':
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


if __name__ == '__main__':
	m = Map()
	a = m.predictPos(m.meters2deg(np.array([-30,-50])), degrees=True)
	print(str_dict(a))
	# import code
	# code.interact(local=locals())