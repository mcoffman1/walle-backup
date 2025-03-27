import matplotlib.pyplot as plt 
import numpy as np
import time

class SpeedCalculator:
	def __init__(self):

		self.scaler = 4
		self.xmaxspeed = 40
		self.ymaxspeed = 40
		self.screenheight = 340
		self.screenwidth = 640
		self.xspeed = 50
		self.yspeed = 50
		self.xcoes = np.array([])
		self.ycoes = np.array([])

	def calc_speed(self, x, y):

		x = x-(self.screenwidth/2)
		xsquare = x*x
		self.xspeed = (self.xcoes[0] * xsquare) + self.xcoes[2]

		y = y-(self.screenheight/2)
		ysquare = y*y
		self.yspeed = (self.ycoes[0] * ysquare) + self.ycoes[2]

		return self.xspeed, self.yspeed
	
	def plotter(self):
	
		xx = np.array([-self.screenwidth/2, -self.screenwidth/4, self.screenwidth/4, self.screenwidth/2])
		yx = np.array([self.ymaxspeed/self.scaler, self.ymaxspeed, self.ymaxspeed, self.ymaxspeed/self.scaler])

		xy = np.array([-self.screenheight/2, -self.screenheight/4, self.screenheight/4, self.screenheight/2])
		yy = np.array([self.xmaxspeed/self.scaler, self.xmaxspeed, self.xmaxspeed, self.xmaxspeed/self.scaler])
	
		model1 = np.poly1d(np.polyfit(xx, yx, 2))
		model2 = np.poly1d(np.polyfit(xy, yy, 2))
	
		self.xcoes = model1.coefficients
		self.ycoes = model2.coefficients
	
		line1 = np.linspace(-self.screenwidth/2, self.screenwidth/2, 100)
		line2 = np.linspace(-self.screenheight/2, self.screenheight/2, 100)
	
		plt.scatter(xx, yx)
		plt.plot(line1, model1(line1))
	
		plt.scatter(xy, yy)
		plt.plot(line2, model2(line2))
		plt.show()
	
		print(model1)
		print(model2)
	

if __name__=='__main__':
	cc = SpeedCalculator()
	cc.plotter()
	while True:
		x = int(input('xpos:'))
		if x == 0:
			break
		y = int(input('ypos:'))
		print(cc.calc_speed(x, y))
		time.sleep(1)

