"""Helper functions for plotting"""

import numpy as np
import os
import yaml

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

def poly(x, p, order):
	y = 0
	for i in range(order+1):
		y += p[i] * x**i
	return y

def loadYAML(comb: str, variableName, arrayLength=0):
	filename = f"params_{comb}.yaml"
	with open(filename, 'r') as f:
		file = yaml.safe_load(f)
	if arrayLength == 0:
		return file[variableName]
	else:
		variables = []
		for i in range(arrayLength+1):
			variables.append(file[f"{variableName}{i}"])
		return variables

def storeYAML(comb: str, variable, variableName, verbose=False):
	filename = f"params_{comb}.yaml"
	# Check if file already exist -> if so, add/overwrite variable
	file = {}
	if os.path.isfile(filename):
		if verbose: 
			print("Overwriting file")
		with open(filename, 'r') as f:
			file = yaml.safe_load(f)

	if type(variable) is list:
		for i in range(len(variable)):
			file[f"{variableName}{i}"] = float(variable[i])
	else:
		file[variableName] = float(variable)
	
	with open(filename, 'w') as f:
		yaml.dump(file, f)

	if verbose:
		print(f"Stored {variableName} in {filename}")