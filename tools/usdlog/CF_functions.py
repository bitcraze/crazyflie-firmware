# -*- coding: utf-8 -*-
"""
decode: decodes binary logged sensor data from crazyflie2 with uSD-Card-Deck
createConfig: create config file which has to placed on µSD-Card
@author: jsschell
"""
from zlib import crc32
import struct
import numpy as np
import os


def decode(filName):
    # read file as binary
    filObj = open(filName, 'rb')
    filCon = filObj.read()
    filObj.close()
    
    # get file size to forecast output array
    statinfo = os.stat(filName)
    
    # process file header
    setWidth = struct.unpack('B', filCon[:1])
    setNames = []
    idx = 1
    for ii in range(0, setWidth[0]):
        startIdx = idx
        while True:
            if filCon[idx] == ','.encode('ascii')[0]:
                break
            idx += 1
        print(filCon[startIdx:idx], startIdx, idx)
        setNames.append(filCon[startIdx:idx])
        idx += 1
    print("[CRC] of file header:", end="")
    crcVal = crc32(filCon[0:idx+4]) & 0xffffffff
    crcErrors = 0
    if ( crcVal == 0xffffffff):
        print("\tOK\t["+hex(crcVal)+"]")
    else:
        print("\tERROR\t["+hex(crcVal)+"]")
        crcErrors += 1
    offset = idx + 4
    
    # process data sets
    setCon = np.zeros(statinfo.st_size) # upper bound...
    idx = 0
    fmtStr = "<"
    for setName in setNames:
        fmtStr += chr(setName[-2])
    setBytes = struct.calcsize(fmtStr)
    while(offset < len(filCon)):
        setNumber = struct.unpack('B', filCon[offset:offset+1])
        offset += 1
        for ii in range(setNumber[0]):
            setCon[idx:idx+setWidth[0]] = np.array(struct.unpack(fmtStr, filCon[offset:setBytes+offset]))
            offset += setBytes
            idx += setWidth[0]
        crcVal = crc32(filCon[offset-setBytes*setNumber[0]-1:offset+4]) & 0xffffffff
        print("[CRC] of data set:", end="")
        if ( crcVal == 0xffffffff):
            print("\tOK\t["+hex(crcVal)+"]")
        else:
            print("\tERROR\t["+hex(crcVal)+"]")
            crcErrors += 1
        offset += 4
    if (not crcErrors):
        print("[CRC] no errors occurred:\tOK")
    else:
        print("[CRC] {0} errors occurred:\tERROR".format(crcErrors))
    
    # remove not required elements and reshape as matrix
    setCon = np.reshape(setCon[0:idx], (setWidth[0], idx//setWidth[0]), 'f')
    
    # create output dictionary
    output = {}
    for ii in range(setWidth[0]):
        output[setNames[ii][0:-3].decode("utf-8").strip()] = setCon[ii]
    return output
