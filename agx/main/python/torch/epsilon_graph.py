import math
import numpy as np
import matplotlib.pyplot as plt
def normal(mu,sigma):
    def f(x):
        z = 1.0*(x-mu)/sigma
        e = math.e**(-0.5*z**2)
        C = math.sqrt(2*math.pi)*sigma
        return 1.0*e/C
    return f
X = 2
dx = 0.1
R = np.arange(-X,X+dx,dx)
L = list()
sdL = (0.5,1,2,3)
Mu = (0, 0.3, 0.6)
res = set(zip(sdL, Mu))
for sd, _mu in res:
    f = normal(mu=_mu,sigma=sd)
    L.append([f(x) for x in R])
plt.figure(figsize=(16, 8)) 
plt.plot(R,L[0],zorder=1,lw=1.5, color='blue', label = 'arm_3') 
plt.plot(R,L[1],zorder=1,lw=1.5, color='red', label = 'arn_2')
plt.plot(R,L[2],zorder=1,lw=1.5, color='purple', label = 'arm_1')
plt.title("Distribution of action-value function", fontsize=16)
plt.legend(fontsize = 16)
plt.show()

