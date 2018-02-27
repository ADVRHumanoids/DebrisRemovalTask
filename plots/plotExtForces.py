from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

ext = genfromtxt("/tmp/loggerExternalWrench.txt");

time = np.arange(0,ext.shape[0]*0.005,0.005);

#plot external forces
pyplot.plot(time,ext[:,0], label = "x axis");
pyplot.plot(time,ext[:,1], label = "y axis");
pyplot.plot(time,ext[:,2], label = "z axis");

pyplot.title("External forces");
pyplot.legend();
pyplot.show();
