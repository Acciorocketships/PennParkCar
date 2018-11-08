import numpy as np
from math import *

class Planner:

	def __init__(self):
		# Options
		self.planningDist = 1 # How far ahead to place the next waypoint
		self.pDotControl = 1 # Larger values will make wider turns
		self.uFuture = 0.05 # How far into the future to look for pdot
		self.intersectionRadius = 5 # Distance from intersection at which state will switch
		self.desiredOffset = 1 # This distance from the edge will be marked as y=0 and tracked
		# Internal Variables
		self.spline = {'p0': np.array([0,0]),    'p1': np.array([0,0]),
					   'p0dot': np.array([0,0]), 'p1dot': np.array([0,0])}


	def psid(self,localvars,globalvars):
		errThres = 0.1
		uReset = 0.95
		pcurr, pdotcurr = self.currPoint(localvars,globalvars)
		u = invhermite(localvars['pos'],self.spline['p0'],self.spline['p1'],self.spline['p0dot'],self.spline['p1dot'])
		posherm = hermite(self.spline['p0'],self.spline['p1'],self.spline['p0dot'],self.spline['p1dot'],u)
		pdotherm = hermite(self.spline['p0'],self.spline['p1'],self.spline['p0dot'],self.spline['p1dot'],u,1)
		err = np.linalg.norm(hermite(self.spline['p0'],self.spline['p1'],self.spline['p0dot'],self.spline['p1dot'],u) - localvars['pos'])
		if err > errThres:
			# Dynamically change uFuture based on oversteer or understeer
			pdotdesired = pdotherm - self.spline['p0dot']
			angledesired = atan2(pdotdesired[1],pdotdesired[0])
			pdotactual = pdotcurr - self.spline['p0dot']
			angledactual = atan2(pdotactual[1],pdotactual[0])
			if angledesired / angledactual > 1:
				self.uFuture /= 1.1
			else:
				self.uFuture *= 1.1
		if err > errThres or u > uReset:
			# Reset Hermite Spline
			pnext, pdotnext = self.nextPoint(localvars,globalvars)
			self.spline['p0'] = pcurr
			self.spline['p1'] = pnext
			self.spline['p0dot'] = pdotcurr
			self.spline['p1dot'] = pdotnext
			u = 0
		# Steering is the difference in angle between the current and future pdot vector
		pdot = hermite(self.spline['p0'],self.spline['p1'],self.spline['p0dot'],self.spline['p1dot'],u+self.uFuture,1)
		psid = atan2(pdot[1],pdot[0])
		return psid



	def currPoint(self,localvars,globalvars):
		return ( localvars['pos'], np.array([cos(localvars['psi']), sin(localvars['psi'])]) )


	def nextPoint(self,localvars,globalvars):
		if not globalvars['intersection']:
			pWaypoint = np.array([localvars['pos'][0] + self.planningDist, 0])
			pDotWaypoint = self.pDotControl * np.array([1, 0])
		else:
			nextdir = globalvars['nextRoad'] - globalvars['roadEnd']
			nextdir = nextdir / np.linalg.norm(nextdir)
			pWaypoint = globalvars['roadEnd'] + self.intersectionRadius * nextdir
			currpsi = atan2(globalvars['roadEnd'][1],globalvars['roadEnd'][0])
			rot = np.array([[cos(currpsi), -sin(currpsi)],[sin(currpsi), cos(currpsi)]])
			pDotWaypoint = self.intersectionRadius * (rot.T @ nextdir)
		return ( pWaypoint, pDotWaypoint )



def rectifyAngle(angle):
	while angle > pi:
		angle -= 2*pi
	while angle < -pi:
		angle += 2*pi
	return angle


def hermite(p0,p1,p0dot,p1dot,u=0.1,n=0):
	# Formatting
	if isinstance(u,np.ndarray):
		u = u.reshape((-1,))
	p0 = np.array(p0).reshape((-1,))
	p1 = np.array(p1).reshape((-1,))
	p0dot = np.array(p0dot).reshape((-1,))
	p1dot = np.array(p1dot).reshape((-1,))
	# Calculation
	U = np.array([nPr(3,n) * (u ** max(0,3-n)),
				  nPr(2,n) * (u ** max(0,2-n)),
				  nPr(1,n) * (u ** max(0,1-n)),
				  nPr(0,n) * (u ** 0)       ],dtype=np.float64).T
	A = np.array([[2. ,-2., 1., 1.], 
				  [-3., 3.,-2.,-1.], 
				  [0. , 0., 1., 0.], 
				  [1. , 0., 0., 0.]])
	P = np.array([p0,p1,p0dot,p1dot],dtype=np.float64)
	ans = U @ A @ P
	return ans



def invhermite(pos,p0,p1,p0dot,p1dot):
	# The squared distance between P and the point X(t) is the function F(t) = Dot(X(t)-P,X(t)-P).
	# The derivative is F'(t) = 2*Dot(X(t)-P,X'(t)). Computing t values for F'(t) = 0 is a root-finding problem.
	# mat1: X(t)-P   mat2: X'(t)
	pos = np.array(pos).reshape((-1,))

	A = np.array([[2. ,-2., 1., 1.], 
				  [-3., 3.,-2.,-1.], 
				  [0. , 0., 1., 0.], 
				  [1. , 0., 0., 0.]])
	P = np.array([p0,p1,p0dot,p1dot])
	deriv = np.array([[0.,0.,0.,0.],
					  [3.,0.,0.,0.],
					  [0.,2.,0.,0.],
					  [0.,0.,1.,0.]]).T
	mat1 = A @ P
	mat1[3,:] -= pos
	mat2 = deriv @ A @ P

	# Dot product
	polys = []
	for i in range(pos.shape[0]):
		poly = np.polymul(np.poly1d(mat1[:,i]),np.poly1d(mat1[:,i]))
		polys.append(poly)
	poly = sum(polys)

	# Find roots
	polyroots = np.roots(poly)

	# Filter into range [0,1]. Add endpoints to check.
	uposs = [0,1]
	for j in range(polyroots.size):
		if polyroots[j] > 0 and polyroots[j] < 1:
			uposs.append(np.real(polyroots[j]))

	# Choose best root
	bestdist = float('inf')
	ubest = 0
	for i in range(len(uposs)):
		phat = hermite(p0,p1,p0dot,p1dot,uposs[i])
		dist = np.linalg.norm(pos - phat)
		if dist < bestdist:
			bestdist = dist
			ubest = uposs[i]

	return ubest



def nPr(n,r):
	if r > n:
		return 0
	ans = 1
	for k in range(n,max(1,n-r),-1):
		ans = ans * k
	return ans



def testhermite():
	from matplotlib import pyplot as plt
	p0 = np.array([0,0])
	p1 = np.array([2,1])
	p0dot = np.array([-1,1])
	p1dot = np.array([0,1])
	u = np.linspace(0,1,100)
	ans = hermite(p0,p1,p0dot,p1dot,u)

	pos = np.array([0.2,0.35])
	uhat = invhermite(pos,p0,p1,p0dot,p1dot)
	pospred = hermite(p0,p1,p0dot,p1dot,uhat)

	plt.plot(ans[:,0],ans[:,1])
	plt.plot(pos[0],pos[1],'bo')
	plt.plot(pospred[0],pospred[1],'ro')
	plt.axis('scaled')
	plt.show()


if __name__ == '__main__':
	testhermite()
