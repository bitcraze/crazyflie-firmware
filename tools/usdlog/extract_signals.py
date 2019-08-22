import CF_functions as cff
import argparse
import numpy

parser = argparse.ArgumentParser()
parser.add_argument("filename")
args = parser.parse_args()

# decode binary log data
logData = cff.decode(args.filename)

for key in logData.keys():
     a = numpy.asarray(logData[key])
     numpy.savetxt("signals/"+key+".csv", a, delimiter=",")

