from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

inter = genfromtxt("/tmp/loggerInteraction.txt");

time = np.arange(0,stiff.shape[0]*0.005,0.005);

pyplot.plot(time,inter[:,0],label = "Interaction value");

pyplot.title("Interaction Field");
pyplot.legend();
pyplot.show();