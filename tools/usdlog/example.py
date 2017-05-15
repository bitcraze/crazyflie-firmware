# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re

# decode binary log data
logData = cff.decode("test100")

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1;
plotRows = 0;

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k

# get plot config from user
plotGyro = 0
if re.search('gyr', keys):
    inStr = input("plot gyro data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotGyro = 1
        plotRows += 1

plotAccel = 0
if re.search('acc', keys):
    inStr = input("plot accel data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotAccel = 1
        plotRows += 1

plotMag = 0
if re.search('mag', keys):   
    inStr = input("plot magnetometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotMag = 1
        plotRows += 1

plotBaro = 0
if re.search('bar', keys):    
    inStr = input("plot barometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotBaro = 1
        plotRows += 1

plotCtrl = 0
if re.search('crol', keys):
    inStr = input("plot control data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotCtrl = 1
        plotRows += 1

plotStab = 0       
if re.search('srol', keys):
    inStr = input("plot stabilizer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotStab = 1
        plotRows += 1        
    
# current plot for simple subplot usage
plotCurrent = 0;

# new figure
plt.figure(0)

if plotGyro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['gyrx'], '-', label='X')
    plt.plot(logData['tick'], logData['gyry'], '-', label='Y')
    plt.plot(logData['tick'], logData['gyrz'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Gyroscope [°/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 
if plotAccel:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['accx'], '-', label='X')
    plt.plot(logData['tick'], logData['accy'], '-', label='Y')
    plt.plot(logData['tick'], logData['accz'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Accelerometer [g]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 

if plotMag:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['magx'], '-', label='X')
    plt.plot(logData['tick'], logData['magy'], '-', label='Y')
    plt.plot(logData['tick'], logData['magz'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Magnetometer')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotBaro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['pres'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Pressure [hPa]')
    
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['temp'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Temperature [degC]')

if plotCtrl:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['crol'], '-', label='roll')
    plt.plot(logData['tick'], logData['cpit'], '-', label='pitch')
    plt.plot(logData['tick'], logData['cyaw'], '-', label='yaw')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Control')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotStab:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['srol'], '-', label='roll')
    plt.plot(logData['tick'], logData['spit'], '-', label='pitch')
    plt.plot(logData['tick'], logData['syaw'], '-', label='yaw')
    plt.plot(logData['tick'], logData['sthr'], '-', label='thrust')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Stabilizer')
    plt.legend(loc=9, ncol=4, borderaxespad=0.)

plt.show()