from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

ext = genfromtxt("/tmp/loggerExternalWrench.txt");
err = genfromtxt("/tmp/loggerXerr.txt");
stiff = genfromtxt("/tmp/loggerStiffness.txt");
inter = genfromtxt("/tmp/loggerInteraction.txt");

time = np.arange(9*0.005 + 0.001,ext.shape[0]*0.005,0.005);

s1 = np.sin(2*np.pi*time)
s2 = np.sin(4*np.pi*time)

fig = pyplot.figure(1)
fig.subplots_adjust(hspace=.5)

pyplot.subplot(411)
pyplot.title("External forces");
pyplot.plot(time,ext[9:,0], label = r'$F_ext(x)$');
pyplot.plot(time,ext[9:,1], label = r'$F_ext(y)$');
pyplot.plot(time,ext[9:,2], label = r'$F_ext(z)$');
pyplot.xticks([])
pyplot.ylabel(r'$F_{ext}[N]$', fontsize=12)
pyplot.legend();

pyplot.subplot(412)
pyplot.title(r'$\Delta X = X_d - X_a$');
pyplot.plot(time,err[9:,0], label = r'$\Delta X(x)$');
pyplot.plot(time,err[9:,1], label = r'$\Delta X(y)$');
pyplot.plot(time,err[9:,2], label = r'$\Delta X(z)$');
pyplot.xticks([])
pyplot.ylabel(r'$\Delta X[m]$', fontsize=12)
pyplot.legend();

pyplot.subplot(413)
pyplot.title("Cartesian Stiffness");
pyplot.plot(time,stiff[9:,0],label = r'$K_c(x)$');
pyplot.plot(time,stiff[9:,1],label = r'$K_c(y)$');
pyplot.plot(time,stiff[9:,2],label = r'$K_c(z)$');
pyplot.xticks([])
pyplot.ylabel(r'$K_c[N/m]$', fontsize=12)
pyplot.legend();

pyplot.subplot(414)
pyplot.title("Interaction Field");
pyplot.plot(time,inter[9:,0], label = "Interaction value");
pyplot.ylabel("Interaction value", fontsize=12)
pyplot.xlabel('Time [s]', fontsize=18)
pyplot.legend();



pyplot.legend();
pyplot.show();

pyplot.show();