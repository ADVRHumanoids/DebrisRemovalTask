from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

ext = genfromtxt("/tmp/loggerXerr.txt");

time = np.arange(9*0.005 + 0.001,ext.shape[0]*0.005,0.005);

#plot external forces
pyplot.plot(time,ext[9:,0], label = "x axis");
pyplot.plot(time,ext[9:,1], label = "y axis");
pyplot.plot(time,ext[9:,2], label = "z axis");

pyplot.title("Delta X");
pyplot.legend();
pyplot.show();
