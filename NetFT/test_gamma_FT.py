import os
from random import betavariate
import NetFT
import time
import matplotlib.pyplot as plt
import numpy as np

ati_gamma_ip = '192.168.1.1'
ft_sensor = NetFT.Sensor(ati_gamma_ip)

x = [0,0]
y = [0,0]
z = [0,0]
alpha = [0,0]
beta = [0,0]
gamma = [0,0]

plt.ion()
fig,(ax1,ax2) = plt.subplots(2,1)
lin1, = ax1.plot(np.arange(len(x)),x, 'b-' , label = 'x')
lin2, = ax1.plot(np.arange(len(y)),y, 'g-' , label = 'y')
lin3, = ax1.plot(np.arange(len(z)),z, 'r-' , label = 'z')
lin4, = ax2.plot(np.arange(len(alpha)),alpha, 'c-' , label = 'alpha')
lin5, = ax2.plot(np.arange(len(beta)),beta, 'm-' , label = 'beta')
lin6, = ax2.plot(np.arange(len(gamma)),gamma, 'y-' , label = 'gamma')

ax1.legend(loc = 'upper left')
ax2.legend(loc = 'upper left')
ax1.set_autoscale_on(True)
ax2.set_autoscale_on(True)
# ax1.autoscale_view(True,True,True)
    
for i in range(0,int(3e9)):
    result = ft_sensor.getMeasurement()
    time.sleep(0.1)

    x = list(np.append(x,[result[0]]))
    y = list(np.append(y,[result[1]]))
    z = list(np.append(z,[result[2]]))
    alpha = list(np.append(alpha,[result[3]]))
    beta = list(np.append(beta,[result[4]]))
    gamma = list(np.append(gamma,[result[5]]))

    lin1.set_data(np.arange(len(x)),x)
    lin2.set_data(np.arange(len(y)),y)
    lin3.set_data(np.arange(len(z)),z)
    lin4.set_data(np.arange(len(alpha)),alpha)
    lin5.set_data(np.arange(len(beta)),beta)
    lin6.set_data(np.arange(len(gamma)),gamma)
    ax1.relim()
    ax1.autoscale_view(True,True,True)
    ax2.relim()
    ax2.autoscale_view(True,True,True)
    fig.canvas.draw()
    fig.canvas.flush_events()

    print(result)