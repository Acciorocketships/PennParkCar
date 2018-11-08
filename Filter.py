from time import time

class Filter:

	def __init__(self,wc=1):
		self.lowmeas = 0
		self.highmeas = 0
		self.filterval = 0
		self.val = 0
		self.wc = wc
		self.time = time()

	def predict(self,dt=None,update=True):
		if dt is None:
			dt = time() - self.time
		self.time = time()
		alpha = 1 / (1 + self.wc * dt)
		filterval = (alpha) * (self.filterval) + (1-alpha) * (self.lowmeas - self.highmeas)
		if update:
			self.filterval = filterval
		self.val = filterval + self.highmeas
		return self.val


class Integrator:

	def __init__(self):
		self.val = 0
		self.time = time()

	def update(self,meas,dt=None,lowerbound=None,upperbound=None):
		if dt is None:
			dt = time() - self.time
		self.time = time()
		self.val += meas * dt
		if lowerbound is not None:
			self.val = max(self.val,lowerbound)
		if upperbound is not None:
			self.val = min(self.val,upperbound)
		return self.val