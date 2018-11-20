from time import time as Time
from scipy.integrate import simps

class Filter:

	def __init__(self,wc=1):
		self.lowmeas = 0
		self.highmeas = 0
		self.filterval = 0
		self.val = 0
		self.wc = wc
		self.time = Time()

	def predict(self,dt=None,update=True):
		if dt is None:
			dt = Time() - self.time
		self.time = Time()
		alpha = 1 / (1 + self.wc * dt)
		filterval = (alpha) * (self.filterval) + (1-alpha) * (self.lowmeas - self.highmeas)
		if update:
			self.filterval = filterval
		self.val = filterval + self.highmeas
		return self.val


class Integrator:

	def __init__(self):
		self.val = 0
		self.cache = [(None,None),(None,None)] # last 2 (val,dt). times: [t-1, t-2]
		self.time = Time()
		self.offset = 0
		self.numcalib = 0

	def calibrate(self,meas):
		self.offset = (self.offset * self.numcalib + meas) / (self.numcalib + 1)
		self.numcalib += 1

	def update(self,meas,dt=None,lowerbound=None,upperbound=None):
		
		# Get Time
		if dt is None:
			dt = Time() - self.time
		self.time = Time()

		# Undo Offset
		meas = meas - self.offset
		
		# Integrate
		if self.cache[0][0] is None and self.cache[1][0] is None:
			self.val += meas * dt
		elif self.cache[1][0] is None:
			self.val += simps([self.cache[0][0],        meas     ],
							  [       0        , self.cache[0][1]])
		else:
			self.val += simps([self.cache[1][0], self.cache[0][0],              meas                ],
							  [        0       , self.cache[1][1], self.cache[1][1]+self.cache[0][1]])
		self.cache[1] = self.cache[0]
		self.cache[0] = (meas,dt)

		# Cap
		if lowerbound is not None:
			self.val = max(self.val,lowerbound)
		if upperbound is not None:
			self.val = min(self.val,upperbound)

		print("offset:", offset, "meas:", meas, "val:", self.val)
		return self.val


if __name__ == '__main__':
	f = Integrator()
	import code; code.interact(local=locals())