from matplotlib import pyplot;
from pylab import genfromtxt;  
import numpy as np;

ext = genfromtxt("/home/user/Desktop/logs/trial8/loggerExternalWrench.txt");
err = genfromtxt("/home/user/Desktop/logs/trial8/loggerXerr.txt");
stiff = genfromtxt("/home/user/Desktop/logs/trial8/loggerStiffness.txt");
fsminter = genfromtxt("/home/user/Desktop/logs/trial8/loggerFSMInteraction.txt");
interfield = genfromtxt("/home/user/Desktop/logs/trial8/loggerInteractionField.txt");
overinter = genfromtxt("/home/user/Desktop/logs/trial8/loggerOverallInteraction.txt");


initsamples = 940;
time = np.arange(0.0,(ext.shape[0]*0.005)-4.7,0.005);
print(ext.shape[0]);
print("aa");
fig = pyplot.figure(1)
fig.subplots_adjust(hspace=.5)

pyplot.subplot(611)
pyplot.title("External forces");
pyplot.plot(time,ext[initsamples:,0], label = r'$F_{ext}(x)$');
pyplot.plot(time,ext[initsamples:,1], label = r'$F_{ext}(y)$');
pyplot.plot(time,ext[initsamples:,2], label = r'$F_{ext}(z)$');
#pyplot.xticks([])
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
pyplot.grid();
pyplot.ylabel(r'$F_{ext} \; [N]$', fontsize=12)
pyplot.legend(loc=3);

pyplot.subplot(612)
pyplot.title(r'$\Delta X = X_d - X_a$');
pyplot.plot(time,err[initsamples:,0], label = r'$\Delta X(x)$');
pyplot.plot(time,err[initsamples:,1], label = r'$\Delta X(y)$');
pyplot.plot(time,err[initsamples:,2], label = r'$\Delta X(z)$');
#pyplot.xticks([]);
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
pyplot.grid();
pyplot.ylabel(r'$\Delta X \; [m]$', fontsize=12)
pyplot.legend(loc=2);

pyplot.subplot(613)
pyplot.title("Cartesian Stiffness");
pyplot.plot(time,stiff[initsamples:,0],label = r'$K_c(x)$');
pyplot.plot(time,stiff[initsamples:,1],label = r'$K_c(y)$');
pyplot.plot(time,stiff[initsamples:,2],label = r'$K_c(z)$');
#pyplot.xticks([]);
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
pyplot.grid();
pyplot.ylabel(r'$K_c \; [N/m]$', fontsize=12)
pyplot.legend(loc=2);

pyplot.subplot(614)
pyplot.title("FSM Interaction value");
pyplot.plot(time,fsminter[initsamples:,0]);
#pyplot.ylabel("Interaction\nvalue", fontsize=12)
#pyplot.xticks([]);
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
#pyplot.grid();
#pyplot.xlabel('Time [s]', fontsize=15)
#pyplot.legend(loc=2);

pyplot.subplot(615)
pyplot.title("Interaction Field value");
pyplot.plot(time,interfield[initsamples:,0]);
#pyplot.ylabel("Interaction\nvalue", fontsize=12)
#pyplot.xticks([]);
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
#pyplot.grid();
#pyplot.xlabel('Time [s]', fontsize=15)
#pyplot.legend(loc=2);

pyplot.subplot(616)
pyplot.title("Interaction expectancy value");
pyplot.plot(time,overinter[initsamples:,0]);
#pyplot.ylabel("Interaction\nvalue", fontsize=12)
pyplot.xticks(np.arange(min(time), max(time)+1, 1),fontsize=9)
#pyplot.grid();
#plt.yticks(0,1);
pyplot.xlabel('Time [s]', fontsize=15)
#pyplot.legend(loc=2);


pyplot.show();

pyplot.show();