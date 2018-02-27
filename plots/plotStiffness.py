from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

stiff = genfromtxt("/tmp/loggerStiffness.txt");

time = np.arange(0,stiff.shape[0]*0.005,0.005);

pyplot.plot(time,stiff[:,0],label = "Stiffness x");
pyplot.plot(time,stiff[:,1],label = "Stiffness y");
pyplot.plot(time,stiff[:,2],label = "Stiffness z");

pyplot.title("Stiffness");
pyplot.legend();
pyplot.show();