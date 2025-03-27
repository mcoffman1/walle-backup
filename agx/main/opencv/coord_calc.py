import matplotlib.pyplot as plt 
import numpy as np
import time

class coord_calculator:
	def __init__(self):

		self.scaler = 5
		self.xinc = 5
		self.yinc = 5
		self.screenheight = 640
		self.screenwidth = 360
		self.xcoes = np.array([0,0,0,0])
		self.ycoes = np.array([0,0,0,0])
		self.posx = 0.0
		self.posy = 60.0

	def coords(self, x, y):

		x = x-(self.screenheight/2)
		xcube = x*x*x
		xsquare = x*x
		x = (self.xcoes[0] * xcube) + (self.xcoes[1] * xsquare) + (self.xcoes[2] * x)
		self.posx = self.posx + x

		y = y-(self.screenwidth/2)
		ycube = y*y*y
		ysquare = y*y
		y = (self.ycoes[0] * ycube) + (self.ycoes[1] * ysquare) + (self.ycoes[2] * y)
		self.posy = self.posy + y

		return self.posx, self.posy
	
	def plotter(self):
		xx = np.array([-self.screenheight/2, -self.screenheight/4, 0, self.screenheight/4, self.screenheight/2])
		yx = np.array([-self.xinc, -self.xinc/self.scaler, 0, self.xinc/self.scaler, self.xinc])
	
		xy = np.array([-self.screenwidth/2, -self.screenwidth/4, 0, self.screenwidth/4, self.screenwidth/2])
		yy = np.array([-self.yinc, -self.yinc/self.scaler, 0, self.yinc/self.scaler, self.yinc])
	
		model1 = np.poly1d(np.polyfit(xx, yx, 3))
		model2 = np.poly1d(np.polyfit(xy, yy, 3))
	
		self.xcoes = model1.coefficients
		self.ycoes = model2.coefficients
	
		line1 = np.linspace(-self.screenheight/2, self.screenheight/2, 100)
		line2 = np.linspace(-self.screenwidth/2, self.screenwidth/2, 100)
	
		plt.scatter(xx, yx)
		plt.plot(line1, model1(line1))
	
		plt.scatter(xy, yy)
		plt.plot(line2, model2(line2))
		plt.show()
	
		#print(self.xcoes)
		#print(self.xcoes)
	



if __name__=='__main__':
	cc = coord_calculator()
	cc.scaler = 5
	cc.xinc = 10
	cc.yinc = 10
	cc.screenheight = 1280
	cc.screenwidth = 1020
	cc.plotter()
	print(cc.coords(960,765))


