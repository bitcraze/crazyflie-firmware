# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

# decode binary log data
logData = cff.decode(args.filename)

# set window background to white
plt.rcParams['figure.facecolor'] = 'w'
    
# number of columns and rows for suplot
plotCols = 1;
plotRows = 1;

# let's see which keys exists in current data set
keys = ""
for k, v in logData.items():
    keys += k

# get plot config from user
plotGyro = 0
if re.search('gyro', keys):
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
if re.search('baro', keys):
    inStr = input("plot barometer data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotBaro = 1
        plotRows += 1

plotCtrl = 0
if re.search('ctrltarget', keys):
    inStr = input("plot control data? ([Y]es / [n]o): ")
    if ((re.search('^[Yy]', inStr)) or (inStr == '')):
        plotCtrl = 1
        plotRows += 1

plotStab = 0
if re.search('stabilizer', keys):
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
    plt.plot(logData['tick'], logData['gyro.x'], '-', label='X')
    plt.plot(logData['tick'], logData['gyro.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['gyro.z'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Gyroscope [°/s]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 
if plotAccel:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['acc.x'], '-', label='X')
    plt.plot(logData['tick'], logData['acc.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['acc.z'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Accelerometer [g]')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)
 

if plotMag:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['mag.x'], '-', label='X')
    plt.plot(logData['tick'], logData['mag.y'], '-', label='Y')
    plt.plot(logData['tick'], logData['mag.z'], '-', label='Z')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Magnetometer')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotBaro:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['baro.pressure'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Pressure [hPa]')
    
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['baro.temp'], '-')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Temperature [degC]')

if plotCtrl:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['ctrltarget.roll'], '-', label='roll')
    plt.plot(logData['tick'], logData['ctrltarget.pitch'], '-', label='pitch')
    plt.plot(logData['tick'], logData['ctrltarget.yaw'], '-', label='yaw')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Control')
    plt.legend(loc=9, ncol=3, borderaxespad=0.)

if plotStab:
    plotCurrent += 1
    plt.subplot(plotRows, plotCols, plotCurrent)
    plt.plot(logData['tick'], logData['stabilizer.roll'], '-', label='roll')
    plt.plot(logData['tick'], logData['stabilizer.pitch'], '-', label='pitch')
    plt.plot(logData['tick'], logData['stabilizer.yaw'], '-', label='yaw')
    plt.plot(logData['tick'], logData['stabilizer.thrust'], '-', label='thrust')
    plt.xlabel('RTOS Ticks')
    plt.ylabel('Stabilizer')
    plt.legend(loc=9, ncol=4, borderaxespad=0.)

plt.show()