"""Helper functions for plotting"""

import numpy as np

def loadFile(filename: str) -> dict:
	rawdata = np.loadtxt(filename, delimiter=',', skiprows=1, ndmin=2)
	data = {}
	time = rawdata[:,0] / 1000 # scale to seconds
	data["time"] = time - time[0] # make time start at 0
	data["thrust"] = rawdata[:,1] # load cell force in [N]
	data["pwm"] = rawdata[:,2]
	data["vbat"] = rawdata[:,3] # in [V]
	data["rpm1"] = rawdata[:,4]
	data["rpm2"] = rawdata[:,5]
	data["rpm3"] = rawdata[:,6]
	data["rpm4"] = rawdata[:,7]
	data["rpm_avg"] = (data["rpm1"]+data["rpm2"]+data["rpm3"]+data["rpm4"])/4
	data["v"] = rawdata[:,8] # in [V]
	data["i"] = rawdata[:,9] # in [A]
	data["p"] = rawdata[:,10] # in [W]
	data["cmd"] = rawdata[:,11] # in [PWM]
	# data["maxThrustVbat"] = rawdata[:,11] # in [V]
	return data

def loadFiles(filenames: list) -> dict:
	data = loadFile(filenames[0])
	for i in range(1, len(filenames)):
		data_new = loadFile(filenames[i])
		# append data
		for k,v in data_new.items():
			data[k] = np.append(data[k], v)
	return data