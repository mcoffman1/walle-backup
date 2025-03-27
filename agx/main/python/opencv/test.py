import matplotlib.pyplot as plt 
import numpy as np
import time

class CoordCalculator:
    def __init__(self):

        self.xscaler = 100
        self.yscaler = 0
        self.xmaxpos = 100
        self.xminpos = -100
        self.ymaxpos = 0
        self.yminpos = 100
        self.screenheight = 340
        self.screenwidth = 640
        self.xpos = 0
        self.ypos = 0
        self.xcoes = np.array([])
        self.ycoes = np.array([])

    #def map(value, start1, stop1, start2, stop2):
        #return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1))

    def mapx(self, value):
        return 100 + (-100 - 100) * ((value - 0) / (self.screenwidth - 0))

    def mapy(self, value):
        return 0 + (100 - 0) * ((value - self.screenheight) / (0 - self.screenheight))

    def calc_pos(self, x, y):

        x = x-(self.screenwidth/2)
        xsquare = x*x
        xcube = x*x*x
        self.xpos = (self.xcoes[0] * xcube) + self.xcoes[1] * xsquare + self.xcoes[2] * x 

        y = y-(self.screenheight/2)
        ysquare = y*y
        ycube = y*y*y
        self.ypos = (self.ycoes[0] * ycube) + self.ycoes[1] * ysquare + self.ycoes[2] * y

        return self.xpos, self.ypos
	
    def plotter(self):
        
        xx = np.array([-self.screenwidth/2, -self.screenwidth/4, 0, self.screenwidth/4, self.screenwidth/2])
        yx = np.array([35, 15, 0, -15, -35])

        xy = np.array([-self.screenheight/2, -self.screenheight/4, 0, self.screenheight/4, self.screenheight/2])
        yy = np.array([75, 60, 50 , 40, 25])
	
        model1 = np.poly1d(np.polyfit(xx, yx, 5))
        model2 = np.poly1d(np.polyfit(xy, yy, 5))
	
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
    while True:
        cc = CoordCalculator()
        cc.plotter()
        x = int(input('x'))
        if x < 0:
            break
        y = int(input('y'))
        print(cc.calc_pos(x, y))
